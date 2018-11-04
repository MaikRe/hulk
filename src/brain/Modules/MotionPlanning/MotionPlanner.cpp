#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "Data/MotionRequest.hpp"
#include "Tools/Math/Angle.hpp"
#include "Tools/Math/Geometry.hpp"
#include "print.h"

#include "MotionPlanner.hpp"

MotionPlanner::MotionPlanner(const ModuleManagerInterface& manager)
  : Module(manager)
  , hybridAlignDistance_(*this, "hybridAlignDistance", [] {})
  , targetAlignDistance_(*this, "targetAlignDistance", [] {})
  , ballOffsetShiftAngle_(*this, "ballOffsetShiftAngle",
                          [this] { ballOffsetShiftAngle_() *= TO_RAD; })
  , ballOffsetDistance_(*this, "ballOffsetDistance", [] {})
  , ballOffsetTargetOrientationTolerance_(
        *this, "ballOffsetTargetOrientationTolerance",
        [this] { ballOffsetTargetOrientationTolerance_() *= TO_RAD; })
  , ballWeight_(*this, "ballWeight", [] {})
  , freeKickAreaWeight_(*this, "freeKickAreaWeight", [] {})
  , robotWeight_(*this, "robotWeight", [] {})
  , fallenRobotWeight_(*this, "fallenRobotWeight", [] {})
  , unknownObstacleWeight_(*this, "unknownObstacleWeight", [] {})
  , totalObstacleWeight_(*this, "totalObstacleWeight", [] {})
  , obstacleDisplacementAngle_(*this, "obstacleDisplacementAngle",
                               [this] { obstacleDisplacementAngle_() *= TO_RAD; })
  , strikerUsesOnlyLocalObstacles_(*this, "strikerUsesOnlyLocalObstacles", [] {})
  , ignoreGoalPostObstacles_(*this, "ignoreGoalPostObstacles", [] {})
  , planForUNSWalking_(*this, "planForUNSWalking", [] {})
  , enableCarefulDribbling_(*this, "enableCarefulDribbling", [] {})
  , carefulDribbleSpeed_(*this, "carefulDribbleSpeed", [] {})
  , carefulDribbleDistanceThreshold_(*this, "carefulDribbleDistanceThreshold", [] {})
  , groundLevelAvoidanceDistance_(*this, "groundLevelAvoidanceDistance", [] {})
  , shoulderLevelAvoidanceDistance_(*this, "shoulderLevelAvoidanceDistance", [] {})
  , dribblingAngleTolerance_(*this, "dribblingAngleTolerance",
                             [this] { dribblingAngleTolerance_() *= TO_RAD; })
  , slowBallApproachFactor_(*this, "slowBallApproachFactor", [] {})
  , motionRequest_(*this)
  , obstacleData_(*this)
  , teamObstacleData_(*this)
  , robotPosition_(*this)
  , ballState_(*this)
  , walkingEngineWalkOutput_(*this)
  , playingRoles_(*this)
  , motionPlannerOutput_(*this)
  , obstacleWeights_()
  , offsetBallTargetReached_(false)
  , ignoreBallObstacle_(false)
{
  ballOffsetShiftAngle_() *= TO_RAD;
  obstacleDisplacementAngle_() *= TO_RAD;
  ballOffsetTargetOrientationTolerance_() *= TO_RAD;
  dribblingAngleTolerance_() *= TO_RAD;

  // Initialize obstacle-weight association
  obstacleWeights_.fill(unknownObstacleWeight_()); // Defaults to unknown obstacle weight
  obstacleWeights_[static_cast<int>(ObstacleType::BALL)] = ballWeight_();
  obstacleWeights_[static_cast<int>(ObstacleType::FREE_KICK_AREA)] = freeKickAreaWeight_();
  obstacleWeights_[static_cast<int>(ObstacleType::ANONYMOUS_ROBOT)] = robotWeight_();
  obstacleWeights_[static_cast<int>(ObstacleType::HOSTILE_ROBOT)] = robotWeight_();
  obstacleWeights_[static_cast<int>(ObstacleType::TEAM_ROBOT)] = robotWeight_();
  obstacleWeights_[static_cast<int>(ObstacleType::FALLEN_ANONYMOUS_ROBOT)] = fallenRobotWeight_();
  obstacleWeights_[static_cast<int>(ObstacleType::FALLEN_HOSTILE_ROBOT)] = fallenRobotWeight_();
  obstacleWeights_[static_cast<int>(ObstacleType::FALLEN_TEAM_ROBOT)] = fallenRobotWeight_();

  if (!totalObstacleWeight_())
  {
    print("MotionPlanner obstacle weight was initialized to 0, all obstacles will be ignored.",
          LogLevel::WARNING);
  }
}

void MotionPlanner::cycle()
{
  // Copy current MotionRequest to MotionPlannerOutput, this way the motionPlanner may modify the
  // request to pass on its results in form of its own output, without modifying the original
  // request. '&*' is needed because the dependency is not a real pointer.
  motionRequest_->copy(&*motionPlannerOutput_);
  // Only perform motionplanning when the robot is walking
  if (motionRequest_->bodyMotion != MotionRequest::BodyMotion::WALK)
  {
    return;
  }
  // When not in velocity mode, calculate the desired rotation and translation
  if (motionPlannerOutput_->walkData.mode != WalkMode::VELOCITY)
  {
    // Create an offset walk target if needed
    if (motionPlannerOutput_->walkData.mode == WalkMode::WALK_BEHIND_BALL)
    {
      // Sets a new offset target away from the ball
      setWalkBehindBallPosition(std::abs(ballOffsetShiftAngle_()));
    }
    else if (motionPlannerOutput_->walkData.mode == WalkMode::DRIBBLE)
    {
      // A small angle ensures a good transtion between walking around the ball and dribbling
      const float dribblingBallOffsetAngle = 2 * TO_RAD;
      setWalkBehindBallPosition(dribblingBallOffsetAngle);
    }
    // Calculate the orientation the robot shoulder achieve.
    motionPlannerOutput_->walkData.target.orientation = calculateRotation();
    // The length of this vector represents the max. velocity limit, not a distance!
    motionPlannerOutput_->walkData.velocity.translation = calculateTranslation();
    // In DRIBBLE mode, change the walking mode to velocity after reaching the offset target
    // waypoint, to avoid braking when getting near the ball
    if (motionPlannerOutput_->walkData.mode == WalkMode::DRIBBLE && offsetBallTargetReached_)
    {
      // Copy the target orientation to the velocity because it is needed in velocity mode
      motionPlannerOutput_->walkData.velocity.rotation =
          motionPlannerOutput_->walkData.target.orientation;
      if (enableCarefulDribbling_() && planForUNSWalking_() &&
          ballState_->position.norm() < carefulDribbleDistanceThreshold_())
      {
        // clip the dribbling velocity since the maximum walking speed might be quite fast
        motionPlannerOutput_->walkData.velocity =
            getClippedDribbleVelocity(motionPlannerOutput_->walkData.velocity);
        assert(!motionPlannerOutput_->walkData.velocity.isPercentage());
      }
      motionPlannerOutput_->walkData.mode = WalkMode::VELOCITY;
    }
  }
  // Serialize and send debug data
  debug().update(mount_, *this);
}

Velocity MotionPlanner::getClippedDribbleVelocity(const Velocity& requestedVelocity) const
{
  const auto absoluteRequestedVelocity =
      requestedVelocity.getAbsolute(walkingEngineWalkOutput_->maxVelocityComponents);
  const float clippedDribbleVelocity =
      std::min(carefulDribbleSpeed_(), absoluteRequestedVelocity.translation.norm());
  const auto walkDirection = requestedVelocity.translation.normalized();
  return {walkDirection * clippedDribbleVelocity, absoluteRequestedVelocity.rotation, false};
}

void MotionPlanner::setWalkBehindBallPosition(float offsetRotationAngle)
{
  // First, the *current* value of the walkData.target gets read
  // and is interpreted as a kickPose attached to the ball.
  const Pose& kickPose = motionRequest_->walkData.target;
  const Vector2f& ballPosition = ballState_->position;
  // Calculate the angle between the ball/robot-line and the direction
  // where the ball should go (indicated by the walkTarget/kickPose orientation)
  const float robot2BallAngle = std::atan2(ballPosition.y(), ballPosition.x());
  const float robot2BallTargetAngle = Angle::angleDiff(robot2BallAngle, kickPose.orientation);
  // The ballTargetDirection is the direction vector pointing to where the ball should move to.
  const Vector2f ballTargetDirection(std::cos(kickPose.orientation),
                                     std::sin(kickPose.orientation));
  // Get a reference to the current walking target to modify it with an offset
  Pose& offsetTarget = motionPlannerOutput_->walkData.target;
  // Constantly apply an offset to the walk target, as long as the targetReached flag is
  // unset (which means the offset hasn't been reached yet; robot isn't close to the ball).
  // The target should be set a offset position that moved back , but it should be set a little
  // bit closer to the side the robot is coming from
  if (!offsetBallTargetReached_)
  {
    // This angle specifies how much the offset target is rotated
    // towards the robot, regardless of the direction that is determined later.
    offsetRotationAngle = std::abs(offsetRotationAngle);
    // Only if the robot is in a certain angle region, rotate the offset position along
    // the ball radius towards the robot. This decreases excessive detouring.
    if (robot2BallTargetAngle > offsetRotationAngle)
    {
      // Line that connects robot position and ball position
      const Vector2f robotToBallVec =
          robotPosition_->robotToField(ballPosition) - robotPosition_->pose.position;
      // Make sure to correctly rotate the walk target towards the left/right side of the field,
      // depending on how the robot and the ball are positioned relative to each other.
      if (robotToBallVec.y() < 0)
      {
        offsetRotationAngle *= -1;
      }
      // Calculate the offset walk target position and also rotate it a little towards the robot.
      // This makes the robot already aim for a path around the ball from farther away.
      offsetTarget.position = ballPosition - Rotation2Df(offsetRotationAngle) *
                                                 ballTargetDirection * ballOffsetDistance_();
    }
    else
    {
      // "Pull" the walk target back as above, but don't rotate it towards the robot
      // since it is already inside an angle region in front of the ball.
      offsetTarget.position = ballPosition - (ballTargetDirection * ballOffsetDistance_());
    }
  }

  // Determine if the ball obstacle should be ignored.
  // It should be ignored if the robot is on the correct side to avoid complications
  // while dribbling. The robot is on the correct side if it is in the half-plane
  // behind the ball away from the enemy side.
  if (robot2BallTargetAngle <= 90 * TO_RAD)
  {
    ignoreBallObstacle_ = true;
  }
  else if (robot2BallTargetAngle > 95 * TO_RAD) // Hysteresis
  {
    ignoreBallObstacle_ = false;
  }

  // In the following, determine if the flag should set by checking if the robot is properly
  // aligned behind the ball. A cone is placed behind the ball and checks
  // are performed to see if the robot is inside the cone and if the robot's
  // orientation matches the target pose's orientation to some degree (pun intended).
  // A hysteresis is used for resetting.

  // Place the apex for the cone check at an offset behind the kick pose or the ball so that
  // it creates a specific opening at the kick pose position. 15cm seems reasonable for now.
  const float opening = 0.15;
  const Vector2f xOffset = (opening / std::tan(dribblingAngleTolerance_())) * ballTargetDirection;
  Vector2f apex;
  if (motionPlannerOutput_->walkData.mode == WalkMode::DRIBBLE)
  {
    apex = ballPosition + xOffset;
  }
  else
  {
    apex = kickPose.position + xOffset;
  }
  // Calculate the angle on the cone between the robot position and the cone axis
  const float axisAngle = Angle::angleDiff(std::atan2(apex.y(), apex.x()), kickPose.orientation);
  // Set the tolerance for the distance check to the
  // hybridAlignDistance to prevent aligning to the offset target
  const float distanceTolerance = hybridAlignDistance_();
  if (!offsetBallTargetReached_)
  {
    if (offsetTarget.position.norm() <= distanceTolerance &&
        (!planForUNSWalking_() ||
         (std::abs(kickPose.orientation) < ballOffsetTargetOrientationTolerance_())) &&
        axisAngle <= dribblingAngleTolerance_())
    {
      offsetBallTargetReached_ = true;
    }
  }
  else
  {
    // Hysteresis to reset the flag based on angle deviation, specifically for dribbling.
    const float angleHysteresis = 10 * TO_RAD;
    // Hysteresis to reset the flag based on distance.
    const float distanceHysteresis = 0.1;
    if (offsetTarget.position.norm() > distanceTolerance + distanceHysteresis ||
        (!planForUNSWalking_() ||
         std::abs(kickPose.orientation) >
             ballOffsetTargetOrientationTolerance_() + angleHysteresis ||
         axisAngle > dribblingAngleTolerance_() + angleHysteresis))
    {
      offsetBallTargetReached_ = false;
    }
  }
}

float MotionPlanner::calculateRotation() const
{
  assert(motionPlannerOutput_->walkData.mode != WalkMode::VELOCITY);
  switch (motionPlannerOutput_->walkData.mode)
  {
    case WalkMode::PATH_WITH_ORIENTATION:
    case WalkMode::DIRECT_WITH_ORIENTATION:
      // Use the target orientation during the whole path in these modes
      return Angle::normalized(motionPlannerOutput_->walkData.target.orientation);
    default:
      return interpolatedAngle();
  }
}

Vector2f MotionPlanner::calculateTranslation()
{
  assert(motionPlannerOutput_->walkData.mode != WalkMode::VELOCITY);
  const Pose& walkTarget = motionPlannerOutput_->walkData.target;
  const float velocityLimit = motionRequest_->walkData.velocity.translation.norm();

  switch (motionPlannerOutput_->walkData.mode)
  {
    case WalkMode::DIRECT:
    case WalkMode::DIRECT_WITH_ORIENTATION:
    {
      // If a direct walking mode was specified, no obstacles avoidance happens
      // and a normalized vector pointing to the target gets returned
      Vector2f outputVector = walkTarget.position;
      return outputVector.normalized() * velocityLimit;
    }
    case WalkMode::DRIBBLE:
    {
      if (offsetBallTargetReached_)
      {
        return dribblingDirection() * velocityLimit;
      }
      return obstacleAvoidanceVector() * velocityLimit;
    }
    case WalkMode::WALK_BEHIND_BALL:
    {
      if (offsetBallTargetReached_)
      {
        // We have reached the forward ball target, now move directly to the ball target
        // and ignore obstacles
        Vector2f outputVector = walkTarget.position.normalized();
        // Factor in the slowBallApproach parameter to avoid overshooting the target pose
        return outputVector * velocityLimit * slowBallApproachFactor_();
      }
      return obstacleAvoidanceVector() * velocityLimit;
    }
    default:
    {
      return obstacleAvoidanceVector() * velocityLimit;
    }
  }
}

Vector2f MotionPlanner::dribblingDirection() const
{
  // Walk directly at the ball, ignoring the obstacles.
  // Also to aim with the foot at the ball that is closest to it.
  // This is done in similar fashion as seen in the BallUtils.

  // First, calculate if the ball destination lies right or left from the ball.
  const Vector2f& ballSource = ballState_->position;
  const Vector2f& ballTarget = motionPlannerOutput_->kickData.ballDestination;
  // TODO remove this after Iran Open 2018
  // The sign of the determinant gives information about the vector alignment
  // const float sourceTargetOrientation =
  //  (ballTarget.x() * ballSource.y() -
  //    ballTarget.y() * ballSource.x()) / sourceToTarget.norm();
  // int sign = sourceTargetOrientation > 0.f ? 1 : -1;
  int sign = 1;

  // Now choose the correct foot
  const Vector2f footSelect1 = (ballTarget - ballSource).normalized() * 0.05f;
  const Vector2f footSelect(sign * footSelect1.y(), -sign * footSelect1.x());
  // Calculate final foot position
  return Vector2f(ballSource + footSelect).normalized();
}

Vector2f MotionPlanner::obstacleAvoidanceVector() const
{ // If no direct walking mode was specified, do obstacle avoidance
  Vector2f targetVec =
      motionPlannerOutput_->walkData.target.position; // Walk target might have been modified above
  targetVec.normalize(); // Get a normalized vector pointing to the target position

  // Holds the superimposed displacement from all obstacles
  Vector2f obstacleDisplacement = Vector2f::Zero();

  // iterate over all obstacles
  // select obstacle model
  const std::vector<const Obstacle*>& obstaclesPtr = getRelevantObstacles();
  for (auto obstacle : obstaclesPtr)
  {
    // Special handling to ignore the ball obstacle while dribbling,
    // depending on robot/ball-alignment and ignoring goal post obstacles if required by
    // configuration.
    if ((obstacle->type == ObstacleType::BALL && ignoreBallObstacle_) ||
        (obstacle->type == ObstacleType::GOAL_POST && ignoreGoalPostObstacles_()))
    {
      continue;
    }

    // Get the displacement vector for each obstacle, then scale it by the obstacle
    // weight before adding it to the superposed total displacement vector.
    try
    {
      auto type = static_cast<int>(obstacle->type);
      obstacleDisplacement += displacementVector(*obstacle) * obstacleWeights_[type];
    }
    catch (const std::exception& e)
    {
      Log(LogLevel::ERROR) << "MotionPlanner: Obstacle was ignored because of an error: "
                           << e.what() << "\n";
    }
  }
  // normalize the displacement vector;
  obstacleDisplacement.normalize();

  // Calculate a weighted combination of target vector and displacement vector
  // to get next direction for the output. Note, that while each obstacle
  // has its own configurable weight, the obstacle weight used here scales
  // the total influence of obstacle displacements.
  Vector2f outputVector = targetVec + obstacleDisplacement * totalObstacleWeight_();
  // normalize the resulting direction vector
  return outputVector.normalized();
}

std::vector<const Obstacle*> MotionPlanner::getRelevantObstacles() const
{
  std::vector<const Obstacle*> obstaclesPtr;
  const bool useOnlyLocalObstacles =
      playingRoles_->role == PlayingRole::STRIKER && strikerUsesOnlyLocalObstacles_();
  if (useOnlyLocalObstacles)
  {
    obstaclesPtr.resize(obstacleData_->obstacles.size());
    std::transform(obstacleData_->obstacles.begin(), obstacleData_->obstacles.end(),
                   obstaclesPtr.begin(),
                   [](auto& obstacle) -> const Obstacle* { return &obstacle; });
  }
  else
  {
    obstaclesPtr.resize(teamObstacleData_->obstacles.size());
    std::transform(teamObstacleData_->obstacles.begin(), teamObstacleData_->obstacles.end(),
                   obstaclesPtr.begin(),
                   [](auto& obstacle) -> const Obstacle* { return &obstacle; });
  }
  return obstaclesPtr;
}

float MotionPlanner::getMinDistToObstacleCenter(const Obstacle& obstacle) const
{
  // is this an obstacle that we can collide with on foot height or is the shoulder height the
  // critical position?
  const bool footObstacle = obstacle.type == ObstacleType::BALL;
  // we have to stay further away from obstacles that reach up to should height
  return obstacle.radius +
         (footObstacle ? groundLevelAvoidanceDistance_() : shoulderLevelAvoidanceDistance_());
}

Vector2f MotionPlanner::displacementVector(const Obstacle& obstacle) const
{
  // A positive dot product means that the obstacle and
  // the walkTarget are on the same sides of the robot, therefore
  // the obstacle might be in front of the robot and relevant for motionPlanning.
  const bool obstacleIsInFront =
      (motionPlannerOutput_->walkData.target.position.dot(obstacle.relativePosition) > 0);
  // All obstacles are modelled as circles. The robot only gets pushed away
  // from an obstacle if it is inside the obstacle's preconfigured avoidance radius.
  const bool obstacleIsNear =
      obstacle.relativePosition.norm() < getMinDistToObstacleCenter(obstacle);
  if (obstacleIsInFront && obstacleIsNear)
  {
    // The walking destination
    const Vector2<float>& targetPosition = motionRequest_->walkData.target.position;
    // Vector pointing in the direction of the obstacle
    const Vector2<float>& obstacleDirection = obstacle.relativePosition.normalized();
    // Determine the relative positioning of the obstacle and the walking destination by evaluating
    // the sign of the determinant of a matrix composed of the two position vectors.
    // detSign = 1 => obstacle left from destination; detSign = -1 => obstacle right from target
    // (detSign = 0 means parallel, left or right doesn't matter in that case)
    const float det =
        targetPosition.x() * obstacleDirection.y() - targetPosition.y() * obstacleDirection.x();
    const int detSign = det > 0 ? -1 : 1;
    // Rotate the vector pointing to the obstacle away from it. Where the correct side
    // is to rotate "away" from the obstacle is determined by the previously calculated sign.
    const float rotAngle = detSign * obstacleDisplacementAngle_();
    return Rotation2Df(rotAngle) * obstacleDirection;
  }
  else
  {
    return Vector2f::Zero();
  }
}

float MotionPlanner::interpolatedAngle() const
{
  assert(hybridAlignDistance_() > targetAlignDistance_());
  // Interpolate between facing the target and adopting the target orientation in other modes.
  const Pose& targetPose = motionPlannerOutput_->walkData.target;
  // The distance from robot origin to target can directly be obtained
  // from coordinates of the target pose because we are using relative coordinates.
  float distanceToTargetPose = targetPose.position.norm();

  // If the distance is to low we return the original orientation to avoid numerical problems.
  if (distanceToTargetPose < 2 * std::numeric_limits<float>::epsilon())
  {
    return targetPose.orientation;
  }

  // Case: Far away from goal -> Face the target position
  float targetFacingFactor = 0;
  if (distanceToTargetPose > hybridAlignDistance_())
  {
    targetFacingFactor = 1;
  }
  // If within goalAlignDistance,
  // or if within hybridAlignDistance AND already close to the targetPose orientation,
  // then stop facing the target position and adopt targetPose orientation directly
  else if ((distanceToTargetPose < targetAlignDistance_()) ||
           (distanceToTargetPose < targetAlignDistance_() +
                                       ((hybridAlignDistance_() - targetAlignDistance_()) / 2.f) &&
            std::abs(targetPose.orientation) < 5 * TO_RAD))
  {
    targetFacingFactor = 0;
  }
  // Else, calculate interpolation value between facing
  // the target and adopting the targetPose orientation.
  // This causes the robot to progressively align to the targetPose orientation
  // the closer it gets to the targetPose.
  else
  {
    targetFacingFactor = (distanceToTargetPose - targetAlignDistance_()) /
                         (hybridAlignDistance_() - targetAlignDistance_());
  }
  // Interpolate between facing the target and adopting the target pose orientation,
  // to calculate the rotation angle to be achieved. To do so, angle deviations
  // are weighted according to the previously calculated targetFacingFactor.
  const Vector2f additiveTerm =
      Vector2f(std::cos(targetPose.orientation), std::sin(targetPose.orientation)) *
      (1.f - targetFacingFactor);
  const Vector2f combinedDirection =
      (targetPose.position * targetFacingFactor / distanceToTargetPose) + additiveTerm;

  return std::atan2(combinedDirection.y(), combinedDirection.x());
}

void MotionPlanner::toValue(Uni::Value& value) const
{
  value = Uni::Value(Uni::ValueType::OBJECT);

  // The walkData velocity always contains the translation to apply instantly
  value["translation"] << motionPlannerOutput_->walkData.velocity.translation;
  // The walkData.target always contains the relative orientation to achieve instantly, regardless
  // of the mode the motionPlanner is in. (while walkData.velocity.rotation is mode dependent)
  value["rotation"] << motionPlannerOutput_->walkData.target.orientation;
  // Show if the offset walk target near the ball has been reached
  value["offsetBallTargetReached"] << offsetBallTargetReached_;
  // Send the current target pose the robot tries to reach.
  // The walkTarget orientation from the motionRequest is being used rather than the one from
  // from the motionPlannerOutput because the motionPlannerOutput contains the
  // orientation to apply instantly while the motionRequest contains the final
  // orientation to be achieved at the target position.
  value["walkTarget"] << Pose(motionPlannerOutput_->walkData.target.position,
                              motionRequest_->walkData.target.orientation);
}
