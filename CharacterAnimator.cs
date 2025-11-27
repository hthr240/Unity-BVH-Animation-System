using System;
using System.Collections;
using System.Collections.Generic;
using static MatrixUtils;
using UnityEngine;

public class CharacterAnimator : MonoBehaviour
{
    public TextAsset BVHFile; // The BVH file that defines the animation and skeleton
    public bool animate; // Indicates whether or not the animation should be running
    public bool interpolate; // Indicates whether or not frames should be interpolated
    [Range(0.01f, 2f)] public float animationSpeed = 1; // Controls the speed of the animation playback

    public BVHData data; // BVH data of the BVHFile will be loaded here
    public float t = 0; // Value used to interpolate the animation between frames
    public float[] currFrameData; // BVH channel data corresponding to the current keyframe
    public float[] nextFrameData; // BVH vhannel data corresponding to the next keyframe

    private String HEAD = "Head";
    private int HEAD_SCALE = 8;
    private int REG_SCALE = 2;
    

    // Start is called before the first frame update
    void Start()
    {
        BVHParser parser = new BVHParser();
        data = parser.Parse(BVHFile);
        CreateJoint(data.rootJoint, Vector3.zero);
    }

    public Matrix4x4 RotateTowardsVector(Vector3 v)
    {
        if (v.magnitude < 0.001f){
            return Matrix4x4.identity;
        }

        Vector3 normV = v.normalized;
        // on XY plane z=0
        float Zangle = -Mathf.Atan2(normV.x, normV.y);
        float Zangle_deg = Zangle * Mathf.Rad2Deg;
        // "(a, b, 0)"

        float Len_xy = Mathf.Sqrt(normV.x * normV.x + normV. y* normV.y);

        // on YZ plane x=0
        float Xangle = Mathf.Atan2(normV.z, Len_xy);
        float Xangle_deg = Xangle * Mathf.Rad2Deg;
        // "(0, b, c)"

        Matrix4x4 Rz = MatrixUtils.RotateZ(Zangle_deg);
        Matrix4x4 Rx = MatrixUtils.RotateX(Xangle_deg);
        Matrix4x4 R = Rz * Rx;

        return R;
    }

    // Creates a Cylinder GameObject between two given points in 3D space
    public GameObject CreateCylinderBetweenPoints(Vector3 p1, Vector3 p2, float diameter)
    {
        // create object
        GameObject cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        
        // create scale matrix
        Vector3 cylinderScale = new Vector3(diameter, (p2-p1).magnitude/2f, diameter);
        Matrix4x4 S = MatrixUtils.Scale(cylinderScale);

        // create rotation matrix
        Vector3 v_rotate = (p2-p1);
        Matrix4x4 R = RotateTowardsVector(v_rotate);

        // create translation matrix
        Matrix4x4 T = MatrixUtils.Translate((p2+p1)/2f);
        
        // create transformation matrix
        Matrix4x4 M = T*R*S;

        // apply transform
        MatrixUtils.ApplyTransform(cylinder, M);

        return cylinder;
    }

    // Creates a GameObject representing a given BVHJoint and recursively creates GameObjects for it's child joints
    public GameObject CreateJoint(BVHJoint joint, Vector3 parentPosition)
    {

        // create the joint ibject
        joint.gameObject = new GameObject(joint.name);
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.parent = joint.gameObject.transform;

        // scale the joint
        Matrix4x4 scale;
        if(joint.name == HEAD){
            scale = Matrix4x4.Scale(Vector3.one * HEAD_SCALE);
        } else {
            scale = Matrix4x4.Scale(Vector3.one * REG_SCALE);
        }
        MatrixUtils.ApplyTransform(sphere, scale);

        // position the joint
        Vector3 position = parentPosition + joint.offset;
        Matrix4x4 m = MatrixUtils.Translate(position);
        MatrixUtils.ApplyTransform(joint.gameObject,m);
    
        foreach (BVHJoint child in joint.children){
            
            CreateJoint(child, position);
            // position the cylinder
            GameObject cylinder = CreateCylinderBetweenPoints(position, child.gameObject.transform.position, 0.6f);
            cylinder.transform.parent = joint.gameObject.transform;
        }
        return joint.gameObject;
    }

    // Transforms BVHJoint according to the keyframe channel data, and recursively transforms its children
    public void TransformJoint(BVHJoint joint, Matrix4x4 parentTransform)
    {
            // Calculate Local Translation
            Matrix4x4 T_local = MatrixUtils.Translate(joint.offset);
            // Calculate Local Rotation
            Matrix4x4 R_local = Matrix4x4.identity;
            Vector3Int rotAxis = joint.rotationChannels;
            // dont need roations for EndSites
            if (!joint.isEndSite)
            {
                // handle interpolation if true
                if (interpolate){
                    // get cur frame and next frame euler angels
                    Vector3 curr_euler = new Vector3(currFrameData[rotAxis.x],
                                                    currFrameData[rotAxis.y],
                                                    currFrameData[rotAxis.z]
                                                    );
                    Vector3 next_euler = new Vector3(nextFrameData[rotAxis.x],
                                                    nextFrameData[rotAxis.y],
                                                    nextFrameData[rotAxis.z]
                                                    );
                    // get quaternion for each frame rotaions
                    Vector4 q_curr = QuaternionUtils.FromEuler(curr_euler, joint.rotationOrder);
                    Vector4 q_next = QuaternionUtils.FromEuler(next_euler, joint.rotationOrder);
                    Vector4 interpulation = QuaternionUtils.Slerp(q_curr, q_next, t);
                    R_local = MatrixUtils.RotateFromQuaternion(interpulation);
                } 
                else
                {
                    // Create the rotation matrices
                    Matrix4x4 Rx = MatrixUtils.RotateX(currFrameData[rotAxis.x]);
                    Matrix4x4 Ry = MatrixUtils.RotateY(currFrameData[rotAxis.y]);
                    Matrix4x4 Rz = MatrixUtils.RotateZ(currFrameData[rotAxis.z]);
                    // Create an array for the ordered rotations
                    Matrix4x4[] orderedRotations = new Matrix4x4[3];
                    // Place the rotation matrices into the array at the index specified by rotationOrder
                    orderedRotations[joint.rotationOrder.x] = Rx;
                    orderedRotations[joint.rotationOrder.y] = Ry;
                    orderedRotations[joint.rotationOrder.z] = Rz;
                    // Multiply the matrices
                    R_local = orderedRotations[0] * orderedRotations[1] * orderedRotations[2];
                }
            }
            // Handle Root Position
            if (joint == data.rootJoint)
            {
                Vector3 cur_pos = new Vector3(
                    currFrameData[joint.positionChannels.x],
                    currFrameData[joint.positionChannels.y],
                    currFrameData[joint.positionChannels.z]
                    );
                // handle interpolation if true
                if (interpolate)
                {
                    Vector3 next_pos = new Vector3(
                    nextFrameData[joint.positionChannels.x],
                    nextFrameData[joint.positionChannels.y],
                    nextFrameData[joint.positionChannels.z]
                    );

                    Vector3 pos_interpulation = Vector3.Lerp(cur_pos, next_pos, t);
                    T_local = MatrixUtils.Translate(joint.offset + pos_interpulation);
                }
                else
                {
                    T_local = MatrixUtils.Translate(joint.offset + cur_pos);
                }
            }
            // Calculate Local Transform
            Matrix4x4 M_local =  T_local * R_local;
            // Calculate Final Global Transform
            Matrix4x4 M_prime = parentTransform * M_local;
            // Apply Final Transform
            MatrixUtils.ApplyTransform(joint.gameObject, M_prime);
            // recurse on all child joints
            foreach (BVHJoint child in joint.children)
            {
                TransformJoint(child, M_prime);
            }
    }

    // Returns the frame nunmber of the BVH animation at a given time
    public int GetFrameNumber(float time)
    {
        // we get the frame number using module operation
        int numFrames = data.numFrames;
        int frameNum = (int)(time/data.frameLength);
        return frameNum%numFrames;

    }

    // Returns the proportion of time elapsed between the last frame and the next one, between 0 and 1
    public float GetFrameIntervalTime(float time)
    {
        // we get the frame interval using the module operation
        float frameLength = data.frameLength;
        float interval_time = time % frameLength;
        return interval_time / frameLength;
    }

    // Update is called once per frame
    void Update()
    {
        float time = Time.time * animationSpeed;
        if (animate)
        {
            int currFrame = GetFrameNumber(time);
            // Your code here
            t = GetFrameIntervalTime(time);
            int nextFrame = (currFrame + 1) % data.numFrames;
            nextFrameData = data.keyframes[nextFrame];
            currFrameData = data.keyframes[currFrame];
            TransformJoint(data.rootJoint, Matrix4x4.identity);
        }
    }
}
