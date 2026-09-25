//! Making the order bevy_rapier creates, changes and removes Rapier objects in independent of how
//! Bevy happens to store entities.
//!
//! Rapier's results depend on the order bodies, colliders and joints were added, changed and
//! removed: it decides which arena slot (and so which handle) each one gets, and the order Rapier
//! keeps them in internally (e.g. where a body that wakes up joins the active set), which is the
//! order the solver works through them in. By default bevy_rapier adds and changes them in query
//! order, which follows Bevy's archetype and table layout, and removes them in the order their
//! components were removed. Two worlds with the same entities can have different layouts
//! depending on their history (e.g. after a rollback, or when one was built from a snapshot of the
//! other), so for deterministic simulation across worlds, enable [`RapierStableOrder`]:
//!
//! - New bodies, colliders and joints are created in ascending [`RapierCreationOrder`], with any
//!   that don't have one created after, in query order.
//! - Changes to them are applied, and removed ones leave Rapier, in ascending handle order.
//!   Handles are part of the simulation state, so they already agree wherever the state does.

use crate::plugin::context::{RapierContextColliders, RapierContextJoints, RapierRigidBodySet};
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use rapier::data::Index;

/// Turns on [`stable_order`](self) processing. See
/// [`RapierPhysicsPlugin::with_stable_order`](crate::plugin::RapierPhysicsPlugin::with_stable_order).
#[derive(Resource, Copy, Clone, Debug, Default)]
pub struct RapierStableOrder;

/// A key that decides the order new Rapier objects are created in when [`RapierStableOrder`] is
/// on. It has to be the same for the same entity in every world that must simulate identically,
/// and unique among the entities created in the same step.
#[derive(Component, Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Reflect)]
pub struct RapierCreationOrder(pub u64);

/// Access to the stable ordering, for the systems that create and remove Rapier objects.
#[derive(SystemParam)]
pub struct StableOrder<'w, 's> {
    enabled: Option<Res<'w, RapierStableOrder>>,
    keys: Query<'w, 's, &'static RapierCreationOrder>,
}

impl StableOrder<'_, '_> {
    /// `items` in creation order: query order unless stable order is on.
    pub fn creation<T>(
        &self,
        items: impl Iterator<Item = T>,
        entity: impl Fn(&T) -> Entity,
    ) -> Vec<T> {
        let mut items: Vec<T> = items.collect();

        if self.enabled.is_some() {
            // Stable, so entities without a key keep their query order after the keyed ones.
            items.sort_by_cached_key(|item| match self.keys.get(entity(item)) {
                Ok(key) => (false, key.0),
                Err(_) => (true, 0),
            });
        }

        items
    }

    /// `items` in the order to apply their changes in: query order unless stable order is on, in
    /// which case in ascending order of their `handle`.
    pub fn changes<T>(
        &self,
        items: impl Iterator<Item = T>,
        handle: impl Fn(&T) -> Index,
    ) -> Vec<T> {
        let mut items: Vec<T> = items.collect();

        if self.enabled.is_some() {
            items.sort_by_cached_key(|item| handle(item).into_raw_parts());
        }

        items
    }

    /// `entities` in removal order: as given unless stable order is on, in which case in
    /// ascending order of the handles `handle` finds for them.
    pub fn removal(
        &self,
        entities: impl Iterator<Item = Entity>,
        handle: impl Fn(Entity) -> Option<Index>,
    ) -> Vec<Entity> {
        let mut entities: Vec<Entity> = entities.collect();

        if self.enabled.is_some() {
            entities.sort_by_cached_key(|&entity| handle(entity).map(Index::into_raw_parts));
        }

        entities
    }
}

/// The body handle of `entity` in whichever context has it.
pub(crate) fn body_index<'a>(
    mut sets: impl Iterator<Item = &'a RapierRigidBodySet>,
    entity: Entity,
) -> Option<Index> {
    sets.find_map(|set| set.entity2body.get(&entity).map(|handle| handle.0))
}

/// The collider handle of `entity` in whichever context has it.
pub(crate) fn collider_index<'a>(
    mut sets: impl Iterator<Item = &'a RapierContextColliders>,
    entity: Entity,
) -> Option<Index> {
    sets.find_map(|set| set.entity2collider.get(&entity).map(|handle| handle.0))
}

/// The impulse joint handle of `entity` in whichever context has it.
pub(crate) fn impulse_joint_index<'a>(
    mut sets: impl Iterator<Item = &'a RapierContextJoints>,
    entity: Entity,
) -> Option<Index> {
    sets.find_map(|set| set.entity2impulse_joint.get(&entity).map(|handle| handle.0))
}

/// The multibody joint handle of `entity` in whichever context has it.
pub(crate) fn multibody_joint_index<'a>(
    mut sets: impl Iterator<Item = &'a RapierContextJoints>,
    entity: Entity,
) -> Option<Index> {
    sets.find_map(|set| {
        set.entity2multibody_joint
            .get(&entity)
            .map(|handle| handle.0)
    })
}
