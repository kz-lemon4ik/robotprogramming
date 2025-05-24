# Inverse Kinematics Research: 2-Link Planar Manipulator

## Mathematical Background

### Forward Kinematics Review

For a 2-link planar manipulator with joint angles θ1 and θ2, and link lengths L1 and L2:

```
x = L1 * cos(θ1) + L2 * cos(θ1 + θ2)
y = L1 * sin(θ1) + L2 * sin(θ1 + θ2)
```

### Inverse Kinematics Problem

Given target coordinates (x, y), find joint angles (θ1, θ2).

## Geometric Solution Method

### Step 1: Calculate θ2 (Elbow Angle)

Using the law of cosines on the triangle formed by the target point, base, and joint 1:

```
d² = x² + y²  (distance to target)
cos(θ2) = (d² - L1² - L2²) / (2 * L1 * L2)
```

For "elbow up" configuration:
```
θ2 = acos((d² - L1² - L2²) / (2 * L1 * L2))
```

### Step 2: Calculate θ1 (Shoulder Angle)

Using geometric relationships:

```
α = atan2(y, x)  (angle to target from base)
β = atan2(L2 * sin(θ2), L1 + L2 * cos(θ2))  (angle offset)
θ1 = α - β
```

## Reachability Analysis

### Workspace Boundaries
- **Inner boundary**: |L1 - L2| (minimum reach)
- **Outer boundary**: L1 + L2 (maximum reach)
- **Reachable condition**: |L1 - L2| ≤ d ≤ L1 + L2

### Singularities
1. **Fully extended**: θ2 = 0, target at maximum reach
2. **Fully retracted**: θ2 = π, target at minimum reach
3. **Origin singularity**: Target at (0,0) when L1 ≠ L2

## Implementation Considerations

### Numerical Stability
- Check reachability before calculation
- Handle acos domain errors
- Use atan2 for quadrant-correct angles
- Validate results within joint limits

### Solution Selection
- "Elbow up" configuration preferred
- Alternative "elbow down": θ2 = -acos(...)
- Consistent solution selection for smooth motion

### Error Handling
- Unreachable targets: return error code
- Invalid input validation
- Graceful degradation for edge cases

## Algorithm Pseudocode

```cpp
bool calculate_joint_angles(double x, double y, double& theta1, double& theta2) {
    // Calculate distance to target
    double d_squared = x*x + y*y;
    double d = sqrt(d_squared);
    
    // Check reachability
    if (d > L1 + L2 || d < abs(L1 - L2)) {
        return false; // Unreachable
    }
    
    // Calculate theta2 (elbow angle)
    double cos_theta2 = (d_squared - L1*L1 - L2*L2) / (2*L1*L2);
    theta2 = acos(cos_theta2);
    
    // Calculate theta1 (shoulder angle)
    double alpha = atan2(y, x);
    double beta = atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
    theta1 = alpha - beta;
    
    return true; // Success
}
```

## Validation Strategy

### Test Cases
1. **Basic positions**: (L1, 0), (0, L1), (L1+L2, 0)
2. **Boundary conditions**: Maximum and minimum reach
3. **Quadrant coverage**: Targets in all four quadrants
4. **Unreachable targets**: Beyond workspace boundaries
5. **Singularities**: Special geometric configurations

### Expected Results
- Consistent angle calculation
- Smooth motion between adjacent targets
- Proper error handling for invalid inputs
- Numerical stability across workspace