using Unity.Entities;
using UnityEngine;

public class CharacterMovementAbilitiesAuthoring : MonoBehaviour
{
    [Header("Movement Settings")]
    public float RunSpeed;
    public float WalkSpeed;
    [Space]
    [Header("Dash Settings")]
    public float DashSpeed;
    public float DashDuration;
    public float DashCooldown;

    public class CharacterMovementAbilitiesBacker : Baker<CharacterMovementAbilitiesAuthoring>
    {
        public override void Bake(CharacterMovementAbilitiesAuthoring abilitiesAuthoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(entity, new MoveAbilityConfig { RunSpeed = abilitiesAuthoring.RunSpeed, WalkSpeed = abilitiesAuthoring.WalkSpeed });
            AddComponent(entity, new DashAbilityConfig { Speed = abilitiesAuthoring.DashSpeed, Duration = abilitiesAuthoring.DashDuration, Cooldown = abilitiesAuthoring.DashCooldown });
            AddComponent(entity, new MoveAbility());
            SetComponentEnabled<MoveAbility>(entity, false);
            AddComponent(entity, new DashAbility());
            SetComponentEnabled<DashAbility>(entity, false);
        }
    }
}