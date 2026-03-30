//! Integration test: verify the stepper survives a discontinuous motor input
//! on the full 13-state QuadrotorSIL model.
//!
//! Steps with zero motors for ~1.2 s (drone sitting on ground, BDF builds up
//! high-order history), then suddenly applies hover-level motor speeds (~760
//! rad/s).  Without the BDF history reset, the solver fails at the
//! discontinuity.

use rumoca::Compiler;
use rumoca_sim::{SimStepper, StepperOptions};

const MODEL_SOURCE: &str = include_str!("../models/QuadrotorSIL.mo");

fn hover_omega() -> f64 {
    let mass = 2.0_f64;
    let g = 9.80665_f64;
    let ct = 8.5e-6_f64;
    (mass * g / (4.0 * ct)).sqrt()
}

fn build_stepper(timeout: f64) -> SimStepper {
    let compiler = Compiler::new().model("QuadrotorSIL");
    let result = compiler
        .compile_str(MODEL_SOURCE, "QuadrotorSIL.mo")
        .expect("model compilation failed");
    SimStepper::new(
        &result.dae,
        StepperOptions {
            rtol: 1e-3,
            atol: 1e-3,
            max_wall_seconds_per_step: Some(timeout),
            ..Default::default()
        },
    )
    .expect("stepper creation failed")
}

#[test]
fn stepper_survives_zero_to_hover_motor_step() {
    let mut stepper = build_stepper(10.0);
    let dt = 0.004;

    // Phase 1: 1.2 s with zero motors — builds BDF history at steady state.
    for i in 0..300 {
        stepper
            .step(dt)
            .unwrap_or_else(|e| panic!("phase 1 step {i} failed at t={:.4}: {e}", stepper.time()));
    }
    assert!(
        (stepper.time() - 1.2).abs() < 0.01,
        "expected t≈1.2, got {}",
        stepper.time()
    );

    // Phase 2: sudden motor input → hover.
    let omega = hover_omega();
    stepper.set_input("omega_m1", omega).unwrap();
    stepper.set_input("omega_m2", omega).unwrap();
    stepper.set_input("omega_m3", omega).unwrap();
    stepper.set_input("omega_m4", omega).unwrap();

    // Step for another 2 s.
    for i in 0..500 {
        stepper.step(dt).unwrap_or_else(|e| {
            panic!(
                "phase 2 step {i} failed at t={:.4}: {e}",
                stepper.time()
            )
        });
    }

    // Should reach ~3.2 s without crashing.
    assert!(
        stepper.time() > 3.0,
        "expected t > 3.0, got {}",
        stepper.time()
    );
}

#[test]
fn stepper_survives_small_motor_perturbation() {
    let mut stepper = build_stepper(10.0);
    let dt = 0.004;

    // 1.2 s with zero motors.
    for i in 0..300 {
        stepper
            .step(dt)
            .unwrap_or_else(|e| panic!("phase 1 step {i} failed: {e}"));
    }

    // Small perturbation: 16.5 rad/s (the exact value from the bug report).
    stepper.set_input("omega_m1", 16.5).unwrap();
    stepper.set_input("omega_m2", 16.5).unwrap();
    stepper.set_input("omega_m3", 16.5).unwrap();
    stepper.set_input("omega_m4", 16.5).unwrap();

    for i in 0..250 {
        stepper.step(dt).unwrap_or_else(|e| {
            panic!(
                "small perturbation step {i} failed at t={:.4}: {e}",
                stepper.time()
            )
        });
    }

    assert!(stepper.time() > 2.0);
}

/// Real-time scenario: tight per-step timeout (50 ms per 4 ms step).
/// Without history reset, the solver may burn its retry budget on the
/// first step after the discontinuity.
#[test]
fn stepper_hover_step_with_realtime_timeout() {
    let mut stepper = build_stepper(0.1); // 100 ms per step — tight for debug builds
    let dt = 0.004;

    // Build up BDF history at steady state.
    for i in 0..300 {
        stepper
            .step(dt)
            .unwrap_or_else(|e| panic!("phase 1 step {i} failed: {e}"));
    }

    // Sudden hover motors.
    let omega = hover_omega();
    stepper.set_input("omega_m1", omega).unwrap();
    stepper.set_input("omega_m2", omega).unwrap();
    stepper.set_input("omega_m3", omega).unwrap();
    stepper.set_input("omega_m4", omega).unwrap();

    for i in 0..500 {
        stepper.step(dt).unwrap_or_else(|e| {
            panic!(
                "realtime step {i} failed at t={:.4}: {e}",
                stepper.time()
            )
        });
    }
    assert!(stepper.time() > 3.0);
}
