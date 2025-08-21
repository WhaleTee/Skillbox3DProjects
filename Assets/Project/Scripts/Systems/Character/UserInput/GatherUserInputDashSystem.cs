using Project.Scripts.Components.Character.Abilities;
using Project.Scripts.Components.Character.Config;
using Unity.Entities;
using UnityEngine.InputSystem;

namespace Project.Scripts.Systems.Character.UserInput
{
  [RequireMatchingQueriesForUpdate]
  [UpdateAfter(typeof(GatherUserInputMovementSystem))]
  public partial class GatherUserInputDashSystem : SystemBase
  {
    private InputAction dashAction;
    private bool dashPressed; 

    protected override void OnStartRunning()
    {
      dashAction = new InputAction("dash", binding: "<Keyboard>/leftShift");
      dashAction.started += ctx => dashPressed = ctx.ReadValue<float>() > 0f;
      dashAction.performed += ctx => dashPressed = ctx.ReadValue<float>() > 0f;
      dashAction.canceled += _ => dashPressed = false;
      dashAction.Enable();
    }

    protected override void OnStopRunning()
    {
      dashAction.Disable();
    }

    protected override void OnUpdate()
    {
      foreach (var (config, entity) in SystemAPI.Query<RefRO<DashAbilityConfig>>()
               .WithPresent<DashAbility>()
               .WithEntityAccess())
      {
        if (!dashPressed || SystemAPI.IsComponentEnabled<DashAbility>(entity)) return;

        var dashAbility = SystemAPI.GetComponent<DashAbility>(entity);
        dashAbility.CooldownLeft = config.ValueRO.Cooldown;
        dashAbility.RemainingTime = config.ValueRO.Duration;
        SystemAPI.SetComponent(entity, dashAbility);
        SystemAPI.SetComponentEnabled<DashAbility>(entity, true);
      }
    }
  }
}