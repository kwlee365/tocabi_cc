# DYROS TOCABI SRBD MPC FRAMEWORK

This project is the **DYROS TOCABI SRBD MPC Framework**, designed to provide a robust and efficient implementation for Model Predictive Control (MPC) in robotics. This document outlines recent updates, known issues, and future tasks.

---

## 🛠️ Updates  
### <2024-01-25, Sat>  
1. **Fixed Reference Horizon Calculations**  
   - Resolved inaccuracies in contact point distance calculations for exception handling cases (e.g., `current_step_num_ = 0, 1, total_step_num_ - 1`).  
   - Affected functions: `getComTrajectory_mpc`, `getContactPointReference`.

2. **Declared Missing Variable**  
   - Added missing declaration for `ZMP_Y_REF_alpha` in the `contactWrenchCalculator` function.

3. **Corrected Friction Cone Constraints**  
   - Updated constraints for the friction cone from `fx = mu * fz` to:  
     - `fx <= mu * fz`  
     - `-mu * fz <= fx`  
   - Implemented these as separate min and max constraints in `computeMPCGradientsHessian`.

---

## 🚧 Known Issues  
1. **COM Vertical Velocity Anomaly**  
   - The vertical velocity of the COM occasionally spikes to approximately **3.9 m/s**.  

2. **MPC Warm-Starting**  
   - Modify the framework to include **MPC Warm-starting** during robot initialization.  

3. **State Variable Adjustments**  
   - Update to allow robot position and velocity inputs as variables.  
     - Positions and velocities must be transformed to the support foot frame.  
     - Severe linear velocity errors require numerical differentiation.  
     - Angular velocities should be derived using Euler angle displacements and their rates through EulerDotToAngVel function.  

4. **SRBD MPC Weight Tuning**  
   - Tune the **SRBD MPC weights**, particularly for the `fZ` reference.  
     - Without proper tuning, the COM tends to fall toward the ground.  
---