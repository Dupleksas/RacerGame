using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class CarControl : MonoBehaviour
{

	public List<AxleInfo> axleInfos; // the information about each individual axle
	public float maxMotorTorque; // maximum torque the motor can apply to wheel
	public float maxSteeringAngle; // maximum steer angle the wheel can have

	// finds the corresponding visual wheel
	// correctly applies the transform
	public void ApplyLocalPositionToVisuals(WheelCollider collider)
	{
		if (collider.transform.childCount == 0) {
			return;
		}
		
		Transform visualWheel = collider.transform.GetChild(0);
		
		Vector3 position;
		Quaternion rotation;
		collider.GetWorldPose(out position, out rotation);
		
		visualWheel.transform.position = position;
		visualWheel.transform.rotation = rotation;
	}
	
	public void FixedUpdate()
	{
		float motor = maxMotorTorque * Input.GetAxis("Vertical");
		float steering = maxSteeringAngle * Input.GetAxis("Horizontal");
		
		foreach (AxleInfo axleInfo in axleInfos)
		{
			if (axleInfo.steering)
			{
				axleInfo.leftWheel.steerAngle = steering;
				axleInfo.rightWheel.steerAngle = steering;
			}
			if (axleInfo.motor) {
				axleInfo.leftWheel.motorTorque = motor;
				axleInfo.rightWheel.motorTorque = motor;
			}

			ApplyLocalPositionToVisuals(axleInfo.leftWheel);
			ApplyLocalPositionToVisuals(axleInfo.rightWheel);
		}
	}

	void Update()
	{
		foreach (AxleInfo axleInfo in axleInfos)
		{
			axleInfo.leftWheelTransform.Rotate(getWheelRotationSpeed(axleInfo.leftWheel.rpm, Time.deltaTime),0,0);
			axleInfo.rightWheelTransform.Rotate(getWheelRotationSpeed(axleInfo.rightWheel.rpm, Time.deltaTime),0,0);
			if (axleInfo.steering)
			{
				axleInfo.leftWheelTransform.localEulerAngles = new Vector3(axleInfo.leftWheelTransform.localEulerAngles.x, axleInfo.leftWheel.steerAngle - axleInfo.leftWheelTransform.localEulerAngles.z, axleInfo.leftWheelTransform.localEulerAngles.z);
				axleInfo.rightWheelTransform.localEulerAngles = new Vector3(axleInfo.rightWheelTransform.localEulerAngles.x, axleInfo.rightWheel.steerAngle - axleInfo.leftWheelTransform.localEulerAngles.z, axleInfo.leftWheelTransform.localEulerAngles.z);
				//wheelFLTrans.localEulerAngles.y = wheelFL.steerAngle - wheelFLTrans.localEulerAngles.z;
				//wheelFRTrans.localEulerAngles = new Vector3(wheelRLTrans.localEulerAngles.x, wheelFR.steerAngle - wheelFRTrans.localEulerAngles.z, wheelFRTrans.localEulerAngles.z);
			}


		}
		WheelPosition();
	}

	public float getWheelRotationSpeed(float wheelSpeed, float deltaTime)
	{
		float rotation = wheelSpeed / 60 * 360 * deltaTime;
		return rotation;
	}


	void WheelPosition()
	{
		RaycastHit hit;
		Vector3 wheelPosition;
		foreach (AxleInfo axleInfo in axleInfos)
		{
			if (Physics.Raycast (axleInfo.leftWheel.transform.position, -axleInfo.leftWheel.transform.up, out hit, axleInfo.leftWheel.radius + axleInfo.leftWheel.suspensionDistance)) {
				wheelPosition = hit.point + axleInfo.leftWheel.transform.up * axleInfo.leftWheel.radius;
			} else {
				wheelPosition = axleInfo.leftWheel.transform.position - axleInfo.leftWheel.transform.up * axleInfo.leftWheel.suspensionDistance;
			}
			axleInfo.leftWheelTransform.position = wheelPosition;
		
			if (Physics.Raycast (axleInfo.rightWheel.transform.position, -axleInfo.rightWheel.transform.up, out hit, axleInfo.rightWheel.radius + axleInfo.rightWheel.suspensionDistance)) {
				wheelPosition = hit.point + axleInfo.rightWheel.transform.up * axleInfo.rightWheel.radius;
			} else {
				wheelPosition = axleInfo.rightWheel.transform.position - axleInfo.rightWheel.transform.up * axleInfo.rightWheel.suspensionDistance;
			}
			axleInfo.rightWheelTransform.position = wheelPosition;
		}
	}

}

[System.Serializable]
public class AxleInfo
{
	public WheelCollider leftWheel;
	public WheelCollider rightWheel;

	public Transform leftWheelTransform;
	public Transform rightWheelTransform;

	public bool motor; // is this wheel attached to motor?
	public bool steering; // does this wheel apply steer angle?
}
