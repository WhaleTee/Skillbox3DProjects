using Project.Scripts.BasicController.Misc;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Project.Scripts.BasicController.Player
{
  [UpdateInGroup(typeof(SimulationSystemGroup), OrderFirst = true)]
  [UpdateBefore(typeof(FixedStepSimulationSystemGroup))]
  public partial class BasicPlayerInputsSystem : SystemBase
  {
    private BasicInputActions InputActions;

    protected override void OnCreate()
    {
      base.OnCreate();

      RequireForUpdate<FixedTickSystem.Singleton>();
      RequireForUpdate(SystemAPI.QueryBuilder().WithAll<BasicPlayer, BasicPlayerInputs>().Build());

      InputActions = new BasicInputActions();
      InputActions.Enable();
      InputActions.DefaultMap.Enable();
    }

    protected override void OnUpdate()
    {
      BasicInputActions.DefaultMapActions defaultMapActions = InputActions.DefaultMap;
      uint tick = SystemAPI.GetSingleton<FixedTickSystem.Singleton>().Tick;

      foreach (var playerInputs in SystemAPI.Query<RefRW<BasicPlayerInputs>>().WithAll<BasicPlayer>())
      {
        playerInputs.ValueRW.MoveInput = Vector2.ClampMagnitude(defaultMapActions.Move.ReadValue<Vector2>(), 1f);
        playerInputs.ValueRW.CameraLookInput = default;
        if (math.lengthsq(defaultMapActions.LookConst.ReadValue<Vector2>()) > math.lengthsq(defaultMapActions.LookDelta.ReadValue<Vector2>()))
        {
          playerInputs.ValueRW.CameraLookInput = defaultMapActions.LookConst.ReadValue<Vector2>() * SystemAPI.Time.DeltaTime;
        }
        else
        {
          playerInputs.ValueRW.CameraLookInput = defaultMapActions.LookDelta.ReadValue<Vector2>();
        }
        playerInputs.ValueRW.CameraZoomInput = defaultMapActions.Scroll.ReadValue<float>();

        if (defaultMapActions.Jump.WasPressedThisFrame())
        {
          playerInputs.ValueRW.JumpPressed.Set(tick);
        }
      }
    }
  }
}