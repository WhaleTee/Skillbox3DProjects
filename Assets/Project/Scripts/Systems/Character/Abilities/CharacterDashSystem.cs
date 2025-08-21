using Project.Scripts.Components.Character.Abilities;
using Project.Scripts.Components.Character.Config;
using Project.Scripts.Systems.Character.UserInput;
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;

namespace Project.Scripts.Systems.Character.Abilities
{
  [UpdateAfter(typeof(GatherUserInputMovementSystem))]
  [UpdateAfter(typeof(GatherUserInputDashSystem))]
  [UpdateBefore(typeof(CharacterMovementSystem))]
  public partial struct CharacterDashSystem : ISystem
  {
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
      var dt = SystemAPI.Time.DeltaTime;
      new CharacterDashJob { DeltaTime = dt }.Schedule();
    }
  }

  [BurstCompile]
  public partial struct CharacterDashJob : IJobEntity
  {
    public float DeltaTime;

    private void Execute(ref MoveAbility moveAbility, EnabledRefRW<DashAbility> enabledDashAbility,
                         ref DashAbility dashAbility, in DashAbilityConfig config)
    {
      if (!moveAbility.Direction.Equals(float3.zero) && dashAbility.RemainingTime > 0)
      {
        dashAbility.RemainingTime -= DeltaTime;
        moveAbility.Direction *= config.Speed;
      }
        
      dashAbility.CooldownLeft -= DeltaTime;
      if (dashAbility.CooldownLeft < 0f) enabledDashAbility.ValueRW = false;
    }
  }
}