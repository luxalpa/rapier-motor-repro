use glam::{vec3, vec4, EulerRot, Mat4, Quat, Vec3, Vec4, Vec4Swizzles};
use rapier3d::na::{Isometry3, UnitQuaternion, Vector3, Vector4};
use rapier3d::prelude::*;

fn main() {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let mut impulse_joint_set = ImpulseJointSet::new();

    /* Create other structures necessary for the simulation. */
    // let gravity = vector![0.0, -9.81, 0.0];
    let gravity = vector![0.0, 0.0, 0.0];
    let integration_parameters = IntegrationParameters {
        max_velocity_iterations: 100,
        ..IntegrationParameters::default()
    };
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();

    // -- Root

    let root_iso = Isometry3::from_parts(
        Vector3::new(2.089463e-8, 87.11984, 15.622739).into(),
        UnitQuaternion::new_unchecked(
            Vector4::new(0.99690366, 1.0277845e-8, 1.197442e-7, 0.07863346).into(),
        ),
    );

    let root_rb = rigid_body_set.insert(
        RigidBodyBuilder::kinematic_position_based()
            .position(root_iso)
            .build(),
    );

    // -- Thigh

    let joint_xform = Mat4 {
        x_axis: vec4(-0.9980216, -0.006397498, -0.06254538, 0.0),
        y_axis: vec4(-0.019888299, -0.9116001, 0.41059646, 0.0),
        z_axis: vec4(-0.059643168, 0.41102806, 0.9096694, 0.0),
        w_axis: vec4(-12.766783, 81.18979, 17.686138, 1.0),
    };

    let thigh_iso: Isometry<Real> = joint_xform.try_into().unwrap();

    // let thigh_iso = Isometry3::from_parts(
    //     Vector3::new(-12.766783, 81.18979, 17.686138).into(),
    //     UnitQuaternion::new_unchecked(
    //         Vector4::new(-0.03126145, 0.21020934, 0.9771503, 0.0034515674).into(),
    //     ),
    // );

    let thigh_rb = rigid_body_set.insert(
        RigidBodyBuilder::dynamic()
            .position(thigh_iso)
            .linear_damping(1.0)
            .angular_damping(100.0)
            .can_sleep(false)
            .build(),
    );

    let point_a = vec3(-13.299082, 49.6889, 28.658503);
    let point_b = vec3(-12.551003, 80.55688, 17.6841);
    let radius = 9.6;

    let point_a = {
        let p = joint_xform.inverse() * Vec4::from((point_a, 1.0));
        point![p.x, p.y, p.z]
    };

    let point_b = {
        let p = joint_xform.inverse() * Vec4::from((point_b, 1.0));
        point![p.x, p.y, p.z]
    };

    collider_set.insert_with_parent(
        ColliderBuilder::new(SharedShape::capsule(point_a, point_b, radius))
            .collision_groups(InteractionGroups {
                filter: Group::NONE,
                memberships: Group::NONE,
            })
            .build(),
        thigh_rb,
        &mut rigid_body_set,
    );

    let local_bind_pose = Mat4 {
        x_axis: vec4(-0.9980216, -0.0034874808, 0.06277467, 0.0),
        y_axis: vec4(-0.019888237, 0.9647001, -0.2625982, 0.0),
        z_axis: vec4(-0.059642937, -0.26332715, -0.9628609, 0.0),
        w_axis: vec4(-12.766783, 6.1802216, -1.1081715, 1.0),
    };

    let p1 = local_bind_pose * Vec4::from((Vec3::ZERO, 1.0));

    let (rot_x, rot_y, rot_z) = Quat::from_mat4(&local_bind_pose).to_euler(EulerRot::XYZ);

    let stiffness = 10000.0;
    let damping = 1000.0;

    impulse_joint_set.insert(
        root_rb,
        thigh_rb,
        SphericalJointBuilder::new()
            .local_anchor1(p1.xyz().into())
            .local_anchor2(point![0.0, 0.0, 0.0])
            .contacts_enabled(false)
            .motor_position(JointAxis::AngX, rot_x / 2.0, stiffness, damping)
            .motor_position(JointAxis::AngY, rot_y / 2.0, stiffness, damping)
            .motor_position(JointAxis::AngZ, rot_z / 2.0, stiffness, damping)
            .build(),
        true,
    );

    const MAX_ITERATIONS: i64 = 1000000;

    for _ in 0..MAX_ITERATIONS {
        let prev_pos = *rigid_body_set.get(thigh_rb).unwrap().position();

        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut rigid_body_set,
            &mut collider_set,
            &mut impulse_joint_set,
            &mut multibody_joint_set,
            &mut ccd_solver,
            &(),
            &(),
        );

        let pos = *rigid_body_set.get(thigh_rb).unwrap().position();

        let change = (prev_pos.translation.vector - pos.translation.vector).magnitude();

        // println!(
        //     "pos: {:?}",
        //     change
        // );

        if change < 0.01 {
            println!("Converged!");
            break;
        }
    }
}
