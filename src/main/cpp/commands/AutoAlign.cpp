#include "commands/AutoAlign.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <Constants.h>

AutoAlign::AutoAlign(DriveSubsystem* drive, VisionSubsystem* vision, frc::Joystick* joystick)
    :m_drive(drive), m_vision(vision), m_joystick(joystick){
  AddRequirements(drive);
  AddRequirements(vision);
  rotationPid.EnableContinuousInput(0,360);
}

void AutoAlign::Initialize() {
 
  // double distance = m_vision->getDistance(12);
  // double xPose = m_drive->GetEstimatedPose().X().value();
  // double setPoint = xPose + ((distance - 5) / 39.37);
}




void AutoAlign::Execute() {
    m_drive->AutoAlign(m_vision->getTY(),0,0);

    // double xspeed = centerPid.Calculate(m_vision->getXmeters(), 0.5);
    // double yspeed = alignPid.Calculate(m_vision->getYmeters(), 0.1);
    // double rotspeed = rotationPid.Calculate(m_vision->getZAngle(), 179);

    // m_drive->Drive(units::meters_per_second_t(frc::ApplyDeadband(yspeed,0.1)),units::meters_per_second_t(frc::ApplyDeadband(-xspeed,0.07)),units::radians_per_second_t{frc::ApplyDeadband(rotspeed,0.075)},false);

  
  // if(m_vision->seeTarget()){
  
  //   frc::Pose2d targetPose = m_vision->targetPoses[m_vision->ClosestTarget().GetFiducialId()];
  //   m_vision->lastTag = m_vision->ClosestTarget().GetFiducialId();
    


  //   if(!(MYABS(targetPose.X().value()-m_drive->m_odometry.GetEstimatedPosition().X().value()) < error && MYABS(targetPose.Y().value()-m_drive->m_odometry.GetEstimatedPosition().Y().value()) < error)){

      
  //     double Xspeed = centerPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().X().value(), targetPose.X().value());
  //     double Yspeed = alignPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Y().value(), targetPose.Y().value());
  //     double rotationSpeed = rotationPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Rotation().Degrees().value(), targetPose.Rotation().Degrees().value());

  //     std::clamp(Xspeed, -1.0, 1.0);
  //     std::clamp(Yspeed,-1.0,1.0);
  //     // Xspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);
  //     // Yspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);
  //     // double xClamp = 0.8;
  //     // double yClamp = 0.8;
  //     // double rotClamp = 0.5;
  //     // if(Xspeed > xClamp){
  //     //   Xspeed=xClamp;
  //     // }
  //     // else if(Xspeed < -xClamp){
  //     //   Xspeed=-xClamp;
  //     // }

  //     // if(Yspeed > yClamp){
  //     //   Yspeed=yClamp;
  //     // }
  //     // else if(Yspeed < -yClamp){
  //     //   Yspeed=-yClamp;
  //     // }

  //     // if(rotationSpeed > rotClamp){
  //     //   rotationSpeed=rotClamp;
  //     // }
  //     // else if(rotationSpeed < -rotClamp){
  //     //   rotationSpeed=-rotClamp;
  //     // }

  //   frc::SmartDashboard::PutNumber("Xspeed", Xspeed);
  //   frc::SmartDashboard::PutNumber("Yspeed", Yspeed);
  //   frc::SmartDashboard::PutNumber("Rotspeed", rotationSpeed);

  //     m_drive->DriveOdo(units::velocity::meters_per_second_t{Xspeed}, 
  //                   units::velocity::meters_per_second_t{Yspeed},
  //                   units::angular_velocity::radians_per_second_t{rotationSpeed}, true);

  //   }

    
  // }
  // else{


   

  //   frc::Pose2d targetPose = m_vision->targetPoses[m_vision->lastTag];

  //   if(!(MYABS(targetPose.X().value()-m_drive->m_odometry.GetEstimatedPosition().X().value()) < error && MYABS(targetPose.Y().value()-m_drive->m_odometry.GetEstimatedPosition().Y().value()) < error)){

  //     double Xspeed = centerPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().X().value(), targetPose.X().value());
  //     double Yspeed = alignPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Y().value(), targetPose.Y().value());
  //     double rotationSpeed = rotationPid.Calculate(m_drive->m_odometry.GetEstimatedPosition().Rotation().Degrees().value(), targetPose.Rotation().Degrees().value());

  //     std::clamp(Xspeed, -1.0, 1.0);
  //     std::clamp(Yspeed,-1.0,1.0);
  //     // Xspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);
  //     // Yspeed = frc::ApplyDeadband(Xspeed,0.0,1.0);
  //     // double xClamp = 0.8;
  //     // double yClamp = 0.8;
  //     // double rotClamp = 0.5;
  //     // if(Xspeed > xClamp){
  //     //   Xspeed=xClamp;
  //     // }
  //     // else if(Xspeed < -xClamp){
  //     //   Xspeed=-xClamp;
  //     // }

  //     // if(Yspeed > yClamp){
  //     //   Yspeed=yClamp;
  //     // }
  //     // else if(Yspeed < -yClamp){
  //     //   Yspeed=-yClamp;
  //     // }

  //     // if(rotationSpeed > rotClamp){
  //     //   rotationSpeed=rotClamp;
  //     // }
  //     // else if(rotationSpeed < -rotClamp){
  //     //   rotationSpeed=-rotClamp;
  //     // }

  //     double x = m_drive->m_odometry.GetEstimatedPosition().X().value();
  //     double y = m_drive->m_odometry.GetEstimatedPosition().Y().value();

      

  //   frc::SmartDashboard::PutNumber("Xspeed", Xspeed);
  //   frc::SmartDashboard::PutNumber("Yspeed", Yspeed);
  //   frc::SmartDashboard::PutNumber("Rotspeed", rotationSpeed);

  //   m_drive->DriveOdo(units::velocity::meters_per_second_t{Xspeed}, 
  //                   units::velocity::meters_per_second_t{Yspeed},
  //                   units::angular_velocity::radians_per_second_t{rotationSpeed}, true);
  //   }


    
    
  // }
}


void AutoAlign::End(bool interrupted) {}




bool AutoAlign::IsFinished() {
  return fabs(m_vision->getTY()) < 0.1 && fabs(m_vision->getTagX()) < 0.01 && fabs(m_vision->getTagY()) < -.25;

  // frc::Pose2d targetPose = m_vision->targetPoses[m_vision->lastTag];
  // ///CANCEL LOGIC
  // double x = m_drive->m_odometry.GetEstimatedPosition().X().value();
  // double y = m_drive->m_odometry.GetEstimatedPosition().Y().value();

  

  // if(m_vision->seeTarget()){
  //   frc::Pose2d targetPose = m_vision->targetPoses[m_vision->ClosestTarget().GetFiducialId()];
  // }
  // else{
  //   frc::Pose2d targetPose = m_vision->targetPoses[m_vision->lastTag];
  // }
  

  // return MYABS(targetPose.X().value()-m_drive->m_odometry.GetEstimatedPosition().X().value()) < error && MYABS(targetPose.Y().value()-m_drive->m_odometry.GetEstimatedPosition().Y().value()) < error;





}
