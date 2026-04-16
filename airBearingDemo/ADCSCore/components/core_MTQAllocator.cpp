#include "core_MTQAllocator.hpp"
#include <cmath>

MTQAllocator::AllocOutput MTQAllocator::allocate(const Vector3& m_body, const Vector3& B_measured) const
{
    AllocOutput out;
    
    // Body dipole components
    Real m_x = m_body(0);  // X-axis dipole [A·m²]
    Real m_y = m_body(1);  // Y-axis dipole [A·m²]
    // m_z is clamped to 0 by B-dot controller, ignored here
    
    // Coil constants (K_coil = N_turns * A_coil)
    Real K_x = Param::Actuators::Coils::K_coil_x;
    Real K_nx = Param::Actuators::Coils::K_coil_nx;
    Real K_y = Param::Actuators::Coils::K_coil_y;
    Real K_ny = Param::Actuators::Coils::K_coil_ny;
    
    // Allocate body dipole to per-face currents
    // m_cmd [A·m²] = K_coil [m²] * I_face [A]
    // => I_face = m_cmd / K_coil
    
    Real I_Xpos = (std::abs(K_x) > static_cast<Real>(1e-9)) ? m_x / K_x : static_cast<Real>(0.0);
    Real I_Xneg = (std::abs(K_nx) > static_cast<Real>(1e-9)) ? -m_x / K_nx : static_cast<Real>(0.0);  // Negative to oppose
    Real I_Ypos = (std::abs(K_y) > static_cast<Real>(1e-9)) ? m_y / K_y : static_cast<Real>(0.0);
    Real I_Yneg = (std::abs(K_ny) > static_cast<Real>(1e-9)) ? -m_y / K_ny : static_cast<Real>(0.0);  // Negative to oppose
    
    // Apply current saturation per face
    Real I_max = Param::Actuators::Coils::I_max;
    I_Xpos = std::max(std::min(I_Xpos, I_max), -I_max);
    I_Xneg = std::max(std::min(I_Xneg, I_max), -I_max);
    I_Ypos = std::max(std::min(I_Ypos, I_max), -I_max);
    I_Yneg = std::max(std::min(I_Yneg, I_max), -I_max);
    
    // Pack per-face currents: [I_Xpos, I_Xneg, I_Ypos, I_Yneg]
    out.face_current(0) = I_Xpos;
    out.face_current(1) = I_Xneg;
    out.face_current(2) = I_Ypos;
    out.face_current(3) = I_Yneg;
    
    // Compute reference field for each face
    // Simplified model: B_face ≈ (μ₀ / 4π) * (2 * dipole_moment) / r³
    // For a calibrated setup with known coil geometry:
    // B_ref = C_B * I where C_B depends on coil geometry
    // 
    // Placeholder: Assume each coil produces ~1 µT/A as a nominal calibration
    // (User calibrates with Helmholtz cage sweep and updates this)
    constexpr Real B_per_amp = static_cast<Real>(1e-5);  // [T/A] placeholder
    
    out.face_b_ref(0) = std::abs(I_Xpos) * B_per_amp;
    out.face_b_ref(1) = std::abs(I_Xneg) * B_per_amp;
    out.face_b_ref(2) = std::abs(I_Ypos) * B_per_amp;
    out.face_b_ref(3) = std::abs(I_Yneg) * B_per_amp;
    
    (void)B_measured;  // Reserved for future adaptive field estimation
    
    return out;
}

Real MTQAllocator::compute_coil_field_estimate(Real coil_current) const
{
    // Placeholder: linear relationship (calibrate with actual coil)
    constexpr Real B_per_amp = static_cast<Real>(1e-5);  // [T/A]
    return std::abs(coil_current) * B_per_amp;
}
