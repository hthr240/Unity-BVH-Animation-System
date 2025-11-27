using System;
using UnityEngine;

public class QuaternionUtils
{
    // The default rotation order of Unity. May be used for testing
    public static readonly Vector3Int UNITY_ROTATION_ORDER = new Vector3Int(1,2,0);

    // Returns the product of 2 given quaternions
    public static Vector4 Multiply(Vector4 q1, Vector4 q2)
    {
        return new Vector4(
            q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
            q1.w*q2.y + q1.y*q2.w + q1.z*q2.x - q1.x*q2.z,
            q1.w*q2.z + q1.z*q2.w + q1.x*q2.y - q1.y*q2.x,
            q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z
        );
    }

    // Returns the conjugate of the given quaternion q
    public static Vector4 Conjugate(Vector4 q)
    {
        return new Vector4(-q.x, -q.y, -q.z, q.w);
    }

    // Returns the Hamilton product of given quaternions q and v
    public static Vector4 HamiltonProduct(Vector4 q, Vector4 v)
    {
        return Multiply(Multiply(q,v), Conjugate(q));
    }

    // Returns a quaternion representing a rotation of theta degrees around the given axis
    public static Vector4 AxisAngle(Vector3 axis, float theta)
    {
        float angle_rad = (theta * Mathf.Deg2Rad)/2;
        Vector3 normAxis = axis.normalized;

        float w = Mathf.Cos(angle_rad);
        float z = Mathf.Sin(angle_rad) * normAxis.z;
        float y = Mathf.Sin(angle_rad) * normAxis.y;
        float x = Mathf.Sin(angle_rad) * normAxis.x;
        return new Vector4(x,y,z,w);

    }

    // Returns a quaternion representing the given Euler angles applied in the given rotation order
    public static Vector4 FromEuler(Vector3 euler, Vector3Int rotationOrder)
    {   
        // create the quaternions for each rotaion
        Vector4 qX = AxisAngle(Vector3.right, euler.x);
        Vector4 qY = AxisAngle(Vector3.up, euler.y);
        Vector4 qZ = AxisAngle(Vector3.forward, euler.z);

        Vector4[] rotations = {qX, qY, qZ};
        Vector4[] order = new Vector4[3];

        // put in order of the input
        order[rotationOrder.x] = rotations[0];
        order[rotationOrder.y] = rotations[1];
        order[rotationOrder.z] = rotations[2];

        return Multiply(order[0], Multiply(order[1], order[2]));
    }

    // Returns a spherically interpolated quaternion between q1 and q2 at time t in [0,1]
    public static Vector4 Slerp(Vector4 q1, Vector4 q2, float t)
    {
			// find the angle
			Vector4 p = q1.normalized;
			Vector4 q = q2.normalized;
			Vector4 q_conP = Multiply(q, Conjugate(p));
			float theta = Mathf.Acos(q_conP.w);

			float denomiter = Mathf.Sin(theta);
			if (denomiter < 1e-8f){
			return p;
			}
			// slerp formula
			float A = Mathf.Sin((1-t) * theta) / denomiter;
			float B = Mathf.Sin(t * theta) / denomiter;
			Vector4 R = A*p + B*q;
			float Rm = R.magnitude;
			return (Rm > 0f) ? (R/Rm) : p;
    }
}