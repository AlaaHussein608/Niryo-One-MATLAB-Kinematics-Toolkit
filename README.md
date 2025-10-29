# Niryo-One-MATLAB-Kinematics-Toolkit

This repository contains a **comprehensive** set of MATLAB scripts and helper functions implementing foundational kinematics tools for the Niryo One robotic arm. The goal is to provide clear, working examples for:

* Direct (forward) kinematics (position/configuration)
* Direct (velocity) model (Jacobian and end‑effector velocity)
* Inverse (position/configuration) kinematics — analytic solution
* Inverse (velocity) kinematics — joint velocities from spatial velocity

All example scripts are written to be readable, well‑commented and easy to adapt to experiments in simulation (CoppeliaSim) or on a real Niryo One.

---

## Contents (files)

* `README.md` (this file)
* `dkm.m` — **helper**: Denavit–Hartenberg homogeneous transform builder (accepts single row or multiple rows)
* `dhMatrix.m` — **helper**: Computes the Denavit–Hartenberg homogeneous transformation matrix for a single robot link given its DH parameters (a, α, d, θ).
* `dkmNiryo.m` — symbolic forward kinematics example (position/configuration)
* `dvmNiryo.m` — direct velocity model: Jacobian computation and end‑effector velocity
* `ikmNiryo.m` — analytic inverse kinematics (wrist decomposition + joint limits filter)
* `ivmNiryo.m` — inverse velocity model using Jacobian pseudoinverse (with simple singularity check)

> The scripts in this repo follow the DH table layout used in the provided examples: each DH row is `[a  alpha  d  theta]` (i.e. standard DH parameters in the order *a, alpha, d, theta*).

---

## `dkmNiryo.m` — forward kinematics (symbolic example)

This script demonstrates use of the DH table for the Niryo One and computes a full transform using symbolic joint variables. Typical workflow in `dkmNiryo.m`:

1. Define symbolic joint variables `t1..t6` and link lengths `l1..l4`.
2. Build the DH table `dhT` in the order `[a alpha d theta]`.
3. Substitute specific joint values into the DH table and call `double(dkm(...))` to obtain numeric 4×4 transform.

**What it gives you:** the homogeneous transform of the end‑effector in base coordinates (pose and orientation).

**Tip:** For visualization or debugging, extract position `T(1:3,4)` and rotation `T(1:3,1:3)` and convert rotation to Euler angles or axis-angle.

---

## `dvmNiryo.m` — direct velocity model (Jacobian)

This script builds the geometric Jacobian using the standard screw axis construction for revolute joints:

* Compute intermediate transforms `T0_i` up to each joint.
* Extract joint axis `Z_i` (third column of rotation) and origin `O_i` (translation part).
* End-effector origin `OE = T0_6(1:3,4)`.
* For revolute joints, the column is `[cross(Z_i, OE - O_i); Z_i]`.

The script stacks columns to form the `6×6` Jacobian `J`. Then given a vector of joint velocities `DTH` it computes spatial twist

```
V = J * DTH
```

`V` is `[vx; vy; vz; wx; wy; wz]` (linear then angular).

**Important notes:**

* The Jacobian depends on the chosen end‑effector frame. The script uses the full transformation down to the actual end‑effector (including the wrist/Tool transforms in the DH table).
* If you need Cartesian (operational) velocity of the flange only, ensure the DH table includes or excludes the tool segment accordingly.

---

## `ivmNiryo.m` — inverse velocity model (joint velocities from twist)

This script computes joint velocities given a desired end‑effector twist `Velocities = [Vx; Vy; Vz; Wx; Wy; Wz]` by using the (Moore-Penrose) pseudoinverse:

```matlab
J_inv = pinv(J);
DTH_velocities = J_inv * Velocities;
```

**Singularities and alternatives:**

* If `rank(J) < 6` the manipulator is at (or near) a singular configuration. The provided script prints a simple detection message.
* For robust numeric behavior near singularities use damped least squares (DLS) or Tikhonov regularization. Example helper:

```matlab
function J_damped = dampedPinv(J, lambda)
    % lambda: small damping factor (e.g. 1e-3..1e-1 depending on units)
    [m,n] = size(J);
    J_damped = (J' * J + lambda^2 * eye(n)) \ J';
end
```

Then compute `DTH = J_damped * Velocities`.

---

## `ikmNiryo.m` — analytic inverse kinematics (wrist decomposition)

`ikmNiryo.m` demonstrates a classical analytic solution for a 6‑DOF arm with a spherical wrist: it separates the problem into two parts:

1. **Wrist position (first three joints):** compute the wrist center position by removing the end‑effector/tool transform `Tw3E` from the target `T`. Solve the planar/arm geometry for the first three joint angles (may produce multiple solutions).
2. **Wrist orientation (last three joints):** compute rotation needed at wrist to match the desired orientation and solve for `t4,t5,t6` (again multiple solutions due to `acos/atan2` branches).

Key implementation details in the script:

* `Tw3E` contains the fixed transform from wrist frame to tool/end-effector frame (taken from DH rows 8–10).
* The code computes candidate solutions for `t1,t2,t3` (up to 4 elbow/shoulder combinations) and then for each candidate computes two wrist configurations (positive/negative pitch) → total up to 8 combos.
* After computing angle candidates, the script filters them using `JointLimits` (converted to degrees) and prints only valid solutions.

---

## Testing & verification

1. Run `dkmNiryo` with symbolic angles, substitute the numeric test angles and check the end‑effector pose.
2. For `ikmNiryo`, after a candidate solution `q` is found, verify with `T_rec = dkm(subs(dhT, {t1..t6}, q))` and compute numerical differences to the target `T` (translation norm and orientation error via e.g. `rotm2axang` or Frobenius norm).
3. For `dvmNiryo` and `ivmNiryo`, test with simple known motions (e.g. set one joint velocity nonzero and verify linear/angular components match expectation).

---

## Common improvements you may want to add

* Add a small numeric tolerance and clamping for acos/asin to avoid NaNs.
* Provide an options struct to `ikmNiryo` to request a particular branch (e.g. elbow-up/elbow-down) and prefer solutions close to current robot pose.
* Add a trajectory generator and integrate with a controller (PI/PD) in joint space for simulation with CoppeliaSim.
* Add collision checking and joint‑limit softening for motion planning.

---
