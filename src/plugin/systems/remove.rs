use crate::dynamics::ImpulseJoint;
use crate::dynamics::MultibodyJoint;
use crate::dynamics::RapierImpulseJointHandle;
use crate::dynamics::RapierMultibodyJointHandle;
use crate::dynamics::RapierRigidBodyHandle;
use crate::dynamics::RigidBody;
use crate::geometry::Collider;
use crate::geometry::ColliderDisabled;
use crate::geometry::RapierColliderHandle;
use crate::plugin::context::{
    RapierContextColliders, RapierContextJoints, RapierContextSimulation, RapierRigidBodySet,
};
use crate::plugin::stable_order::{
    body_index, collider_index, impulse_joint_index, multibody_joint_index, StableOrder,
};
use crate::prelude::MassModifiedEvent;
use crate::prelude::RigidBodyDisabled;
use crate::prelude::Sensor;
use bevy::ecs::query::IterQueryData;
use bevy::prelude::*;

/// System responsible for removing from Rapier the rigid-bodies/colliders/joints which had
/// their related `bevy_rapier` components removed by the user (through component removal or
/// despawn).
pub fn sync_removals(
    mut commands: Commands,
    mut context_writer: Query<(
        &mut RapierContextSimulation,
        &mut RapierContextColliders,
        &mut RapierContextJoints,
        &mut RapierRigidBodySet,
    )>,
    // Sometimes a Remove immediately followed by Add happens. These `q_has_*` queries prevent that immediate Add
    // from being removed by this system by verifying it's still removed.
    (
        q_has_rigidbody_handle,
        q_has_collider_handle,
        q_has_multibody_joint_handle,
        q_has_impulse_joint_handle,
    ): (
        Query<(), With<RapierRigidBodyHandle>>,
        Query<(), With<RapierColliderHandle>>,
        Query<(), With<RapierMultibodyJointHandle>>,
        Query<(), With<RapierImpulseJointHandle>>,
    ),
    mut removed_bodies: RemovedComponents<RapierRigidBodyHandle>,
    mut removed_colliders: RemovedComponents<RapierColliderHandle>,
    mut removed_impulse_joints: RemovedComponents<RapierImpulseJointHandle>,
    mut removed_multibody_joints: RemovedComponents<RapierMultibodyJointHandle>,
    orphan_bodies: Query<Entity, (With<RapierRigidBodyHandle>, Without<RigidBody>)>,
    orphan_colliders: Query<Entity, (With<RapierColliderHandle>, Without<Collider>)>,
    orphan_impulse_joints: Query<Entity, (With<RapierImpulseJointHandle>, Without<ImpulseJoint>)>,
    orphan_multibody_joints: Query<
        Entity,
        (With<RapierMultibodyJointHandle>, Without<MultibodyJoint>),
    >,

    mut removed_sensors: RemovedComponents<Sensor>,
    mut removed_rigid_body_disabled: RemovedComponents<RigidBodyDisabled>,
    mut removed_colliders_disabled: RemovedComponents<ColliderDisabled>,

    mut mass_modified: MessageWriter<MassModifiedEvent>,
    order: StableOrder,
) {
    // Each is looked up before anything is removed, so the order is decided by the handles the
    // entities had coming in.
    let body = |entity| body_index(context_writer.iter().map(|context| context.3), entity);
    let removed_bodies = order.removal(
        removed_bodies
            .read()
            .filter(|e| !q_has_rigidbody_handle.contains(*e)),
        body,
    );
    let orphan_bodies = order.removal(orphan_bodies.iter(), body);

    let collider = |entity| collider_index(context_writer.iter().map(|context| context.1), entity);
    let removed_colliders = order.removal(
        removed_colliders
            .read()
            .filter(|e| !q_has_collider_handle.contains(*e)),
        collider,
    );
    let orphan_colliders = order.removal(orphan_colliders.iter(), collider);

    let impulse_joint =
        |entity| impulse_joint_index(context_writer.iter().map(|context| context.2), entity);
    let removed_impulse_joints = order.removal(
        removed_impulse_joints
            .read()
            .filter(|e| !q_has_impulse_joint_handle.contains(*e)),
        impulse_joint,
    );
    let orphan_impulse_joints = order.removal(orphan_impulse_joints.iter(), impulse_joint);

    let multibody_joint =
        |entity| multibody_joint_index(context_writer.iter().map(|context| context.2), entity);
    let removed_multibody_joints = order.removal(
        removed_multibody_joints
            .read()
            .filter(|e| !q_has_multibody_joint_handle.contains(*e)),
        multibody_joint,
    );
    let orphan_multibody_joints = order.removal(orphan_multibody_joints.iter(), multibody_joint);

    let removed_sensors = order.removal(removed_sensors.read(), collider);
    let removed_colliders_disabled = order.removal(removed_colliders_disabled.read(), collider);
    let removed_rigid_body_disabled = order.removal(removed_rigid_body_disabled.read(), body);

    /*
     * Rigid-bodies removal detection.
     */
    for entity in removed_bodies {
        let Some(((mut context, mut context_colliders, mut joints, mut rigidbody_set), handle)) =
            find_context(&mut context_writer, |res| res.3.entity2body.remove(&entity))
        else {
            continue;
        };
        let context = &mut *context;
        let joints = &mut *joints;

        let _ = rigidbody_set.last_body_transform_set.remove(&handle);
        rigidbody_set.bodies.remove(
            handle,
            &mut context.islands,
            &mut context_colliders.colliders,
            &mut joints.impulse_joints,
            &mut joints.multibody_joints,
            false,
        );
    }

    for entity in orphan_bodies {
        if let Some(((mut context, mut context_colliders, mut joints, mut rigidbody_set), handle)) =
            find_context(&mut context_writer, |res| res.3.entity2body.remove(&entity))
        {
            let context = &mut *context;
            let joints = &mut *joints;
            let _ = rigidbody_set.last_body_transform_set.remove(&handle);
            rigidbody_set.bodies.remove(
                handle,
                &mut context.islands,
                &mut context_colliders.colliders,
                &mut joints.impulse_joints,
                &mut joints.multibody_joints,
                false,
            );
        }
        commands.entity(entity).remove::<RapierRigidBodyHandle>();
    }

    /*
     * Collider removal detection.
     */
    for entity in removed_colliders {
        let Some(((mut context, mut context_colliders, _, mut rigidbody_set), handle)) =
            find_context(&mut context_writer, |res| {
                res.1.entity2collider.remove(&entity)
            })
        else {
            continue;
        };
        let context = &mut *context;
        if let Some(parent) = context_colliders.collider_parent(&rigidbody_set, entity) {
            mass_modified.write(parent.into());
        }

        context_colliders.colliders.remove(
            handle,
            &mut context.islands,
            &mut rigidbody_set.bodies,
            true,
        );
        context.deleted_colliders.insert(handle, entity);
    }

    for entity in orphan_colliders {
        if let Some(((mut context, mut context_colliders, _, mut rigidbody_set), handle)) =
            find_context(&mut context_writer, |res| {
                res.1.entity2collider.remove(&entity)
            })
        {
            let context = &mut *context;
            let context_colliders = &mut *context_colliders;
            if let Some(parent) = context_colliders.collider_parent(&rigidbody_set, entity) {
                mass_modified.write(parent.into());
            }

            context_colliders.colliders.remove(
                handle,
                &mut context.islands,
                &mut rigidbody_set.bodies,
                true,
            );
            context.deleted_colliders.insert(handle, entity);
        }
        commands.entity(entity).remove::<RapierColliderHandle>();
    }

    /*
     * Impulse joint removal detection.
     */
    for entity in removed_impulse_joints {
        let Some(((_, _, mut joints, _), handle)) = find_context(&mut context_writer, |res| {
            res.2.entity2impulse_joint.remove(&entity)
        }) else {
            continue;
        };
        joints.impulse_joints.remove(handle, true);
    }

    for entity in orphan_impulse_joints {
        if let Some(((_, _, mut joints, _), handle)) = find_context(&mut context_writer, |res| {
            res.2.entity2impulse_joint.remove(&entity)
        }) {
            joints.impulse_joints.remove(handle, true);
        }
        commands.entity(entity).remove::<RapierImpulseJointHandle>();
    }

    /*
     * Multibody joint removal detection.
     */
    for entity in removed_multibody_joints {
        let Some(((_, _, mut joints, _), handle)) = find_context(&mut context_writer, |res| {
            res.2.entity2multibody_joint.remove(&entity)
        }) else {
            continue;
        };
        joints.multibody_joints.remove(handle, true);
    }

    for entity in orphan_multibody_joints {
        if let Some(((_, _, mut joints, _), handle)) = find_context(&mut context_writer, |res| {
            res.2.entity2multibody_joint.remove(&entity)
        }) {
            joints.multibody_joints.remove(handle, true);
        }
        commands
            .entity(entity)
            .remove::<RapierMultibodyJointHandle>();
    }

    /*
     * Marker components removal detection.
     */
    for entity in removed_sensors {
        if let Some((mut context, handle)) = find_context(&mut context_writer, |context| {
            context.1.entity2collider.get(&entity).copied()
        }) {
            if let Some(co) = context.1.colliders.get_mut(handle) {
                co.set_sensor(false);
            }
        }
    }

    for entity in removed_colliders_disabled {
        if let Some((mut context, handle)) = find_context(&mut context_writer, |context| {
            context.1.entity2collider.get(&entity).copied()
        }) {
            if let Some(co) = context.1.colliders.get_mut(handle) {
                co.set_enabled(true);
            }
        }
    }

    for entity in removed_rigid_body_disabled {
        if let Some(((_, _, _, mut rigidbody_set), handle)) =
            find_context(&mut context_writer, |res| {
                res.3.entity2body.get(&entity).copied()
            })
        {
            if let Some(rb) = rigidbody_set.bodies.get_mut(handle) {
                rb.set_enabled(true);
            }
        }
    }

    // TODO: what about removing forces?
}

fn find_context<'a, TReturn, TQueryParams: IterQueryData>(
    context_writer: &'a mut Query<TQueryParams>,
    item_finder: impl Fn(&mut TQueryParams::Item<'_, '_>) -> Option<TReturn>,
) -> Option<(TQueryParams::Item<'a, 'a>, TReturn)> {
    let ret: Option<(TQueryParams::Item<'_, '_>, TReturn)> = context_writer
        .iter_mut()
        .find_map(|mut context| item_finder(&mut context).map(|handle| (context, handle)));
    ret
}
