# 3D Character Animation Engine | Unity & C#

A raw C# implementation of a **3D Animation Engine** within Unity. Unlike standard Unity development, this project manually handles **Matrix Transformations**, **Forward Kinematics**, and **Quaternion Interpolation (SLERP)** from scratch to render skeletal animations parsed from BVH files.

## 🚀 Key Features
* **Custom Physics/Math Engine:** Implemented a linear algebra library (`MatrixUtils.cs`) to handle 4x4 Homogeneous Transformation Matrices (Rotation, Translation, Scaling) without using Unity's built-in `transform` methods.
* **Quaternion Math:** Developed a custom Quaternion arithmetic library (`QuaternionUtils.cs`) including **SLERP (Spherical Linear Interpolation)** for smooth rotational transitions between keyframes.
* **BVH Parser:** Built a parser to read standard **Biovision Hierarchy (BVH)** files, constructing a recursive skeletal hierarchy of joints and bones.
* **Forward Kinematics:** Implemented the FK algorithm to propagate local joint transformations down the skeletal hierarchy to calculate world-space positions.

## 🛠️ Tech Stack
* **Engine:** Unity (C#)
* **Math:** Linear Algebra (Homogeneous Coordinates, Affine Transformations)
* **Animation:** Forward Kinematics, Interpolation, Skeleton Hierarchies
* **Data Format:** BVH (Biovision Hierarchy)

## 📂 Project Structure
* `CharacterAnimator.cs`: The main engine controller. Manages the animation loop, time interpolation ($t$), and frame updates.
* `MatrixUtils.cs`: A static library implementing 4x4 Matrix multiplication and transformations from scratch.
* `QuaternionUtils.cs`: Implements complex rotation logic, including converting Euler angles to Quaternions and performing SLERP.
* `BVHParser.cs`: Reads raw binary/text BVH data and constructs the `BVHJoint` object graph.
