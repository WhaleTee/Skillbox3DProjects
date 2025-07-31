using Unity.Burst;
using Unity.Entities;

[UpdateInGroup(typeof(SimulationSystemGroup))]
[UpdateAfter(typeof(FixedStepSimulationSystemGroup))]
[BurstCompile]
public partial struct BasicPlayerVariableStepControlSystem : ISystem
{
  [BurstCompile]
  public void OnCreate(ref SystemState state)
  {
    state.RequireForUpdate(SystemAPI.QueryBuilder().WithAll<BasicPlayer, BasicPlayerInputs>().Build());
  }

  [BurstCompile]
  public void OnUpdate(ref SystemState state)
  {
    foreach (var (playerInputs, player) in SystemAPI.Query<RefRO<BasicPlayerInputs>, RefRO<BasicPlayer>>().WithAll<Simulate>())
    {
      if (SystemAPI.HasComponent<OrbitCameraControl>(player.ValueRO.ControlledCamera))
      {
        OrbitCameraControl cameraControl = SystemAPI.GetComponent<OrbitCameraControl>(player.ValueRO.ControlledCamera);

        cameraControl.FollowedCharacterEntity = player.ValueRO.ControlledCharacter;
        cameraControl.LookDegreesDelta = playerInputs.ValueRO.CameraLookInput;
        cameraControl.ZoomDelta = playerInputs.ValueRO.CameraZoomInput;

        SystemAPI.SetComponent(player.ValueRO.ControlledCamera, cameraControl);
      }
    }
  }
}