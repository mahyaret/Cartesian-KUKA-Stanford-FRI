// CartTraj.cpp : Defines the entry point for the console application.
//

#include "CartTraj.h"

//*******************************************************************************************
// main()
//
int main(int argc, char *argv[])
{
	unsigned int				CycleCounter = 0
		, i = 0;

	int							ResultValue = 0;

	float						CommandedForcesAndTorques[NUMBER_OF_CART_DOFS]
		, CommandedStiffness[NUMBER_OF_CART_DOFS]
		, CommandedDamping[NUMBER_OF_CART_DOFS]
		, EstimatedExternalCartForcesAndTorques[NUMBER_OF_CART_DOFS]
		, CommandedPose[NUMBER_OF_FRAME_ELEMENTS]
		, MeasuredPose[NUMBER_OF_FRAME_ELEMENTS]
		, JointValuesInRad[NUMBER_OF_JOINTS]
		, MeasuredJointTorques[NUMBER_OF_JOINTS]
		, kukaHomePose[NUMBER_OF_FRAME_ELEMENTS] = {  0.0, 1.0, 0.0, 0.0
													, 1.0, 0.0, 0.0, 0.5
													, 0.0, 0.0, -1.0, 0.5 };

	LWRCartImpedanceController	*Robot;

	TypeIRML					*RML = NULL;
	TypeIRMLInputParameters		*IP = NULL;
	TypeIRMLOutputParameters	*OP = NULL;

	RML = new TypeIRML(NUMBER_OF_FRAME_ELEMENTS, 0.002);

	IP = new TypeIRMLInputParameters(NUMBER_OF_FRAME_ELEMENTS);

	OP = new TypeIRMLOutputParameters(NUMBER_OF_FRAME_ELEMENTS);

	Robot = new LWRCartImpedanceController("Z:\\Dropbox\\Ph.D\\Hardware-Software\\KUKA\\fril\\FRILibrary\\etc\\980039-FRI-Driver.init");

	fprintf(stdout, "RobotCartImpedanceController object created. Starting the robot...\n");

	ResultValue = Robot->StartRobot();

	if (ResultValue == EOK)
	{
		fprintf(stdout, "Robot successfully started.\n");
	}
	else
	{
		fprintf(stderr, "ERROR, could not start robot: %s\n", strerror(ResultValue));
	}

	fprintf(stdout, "Current system state:\n%s\n", Robot->GetCompleteRobotStateAndInformation());

	for (i = 0; i < NUMBER_OF_CART_DOFS; i++)
	{
		CommandedStiffness[i] = (float)250.0 * ((i <= 2) ? (10.0) : (1.0));
		CommandedDamping[i] = (float)0.7;
		CommandedForcesAndTorques[i] = (float)0.0;
	}

	Robot->GetMeasuredCartPose(MeasuredPose);

	for (i = 0; i < NUMBER_OF_FRAME_ELEMENTS; i++)
	{
		IP->CurrentPosition->VecData[i] = (double)(MeasuredPose[i]);
		IP->TargetPosition->VecData[i] = (double)(kukaHomePose[i]);
		IP->MaxVelocity->VecData[i] = (double)2;
		IP->MaxAcceleration->VecData[i] = (double)1;
		IP->SelectionVector->VecData[i] = true;
	}

	ResultValue = TypeIRML::RML_WORKING;

	while ((Robot->IsMachineOK()) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
	{
		Robot->WaitForKRCTick();

		ResultValue = RML->GetNextMotionState_Position(*IP, OP);

		if ((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
		{
			printf("CartTraj: ERROR during trajectory generation (%d).", ResultValue);
		}

		for (i = 0; i < NUMBER_OF_FRAME_ELEMENTS; i++)
		{
			CommandedPose[i] = (float)(OP->NewPosition->VecData[i]);
		}
		
		Robot->GetMeasuredJointTorques(MeasuredJointTorques);
		Robot->GetMeasuredJointPositions(JointValuesInRad);
		Robot->GetMeasuredCartPose(MeasuredPose);
		Robot->GetEstimatedExternalCartForcesAndTorques(EstimatedExternalCartForcesAndTorques);

		Robot->SetCommandedCartStiffness(CommandedStiffness);
		Robot->SetCommandedCartDamping(CommandedDamping);
		Robot->SetCommandedCartForcesAndTorques(CommandedForcesAndTorques);
		Robot->SetCommandedCartPose	(CommandedPose);

		*(IP->CurrentPosition) = *(OP->NewPosition);
		*(IP->CurrentVelocity) = *(OP->NewVelocity);
	}

	fprintf(stdout, "Stopping the robot...\n");
	ResultValue = Robot->StopRobot();

	if (ResultValue != EOK)
	{
		fprintf(stderr, "An error occurred during stopping the robot...\n");
	}
	else
	{
		fprintf(stdout, "Robot successfully stopped.\n");
	}

	fprintf(stdout, "Deleting the object...\n");
	delete Robot;
	fprintf(stdout, "Object deleted...\n");

	return(EXIT_SUCCESS);
}


