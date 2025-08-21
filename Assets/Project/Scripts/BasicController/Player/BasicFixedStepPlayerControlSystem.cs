using Project.Scripts.BasicController.Camera;
using Project.Scripts.BasicController.Character;
using Project.Scripts.BasicController.Misc;
using Unity.Burst;
using Unity.CharacterController;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Project.Scripts.BasicController.Player
{
  [UpdateInGroup(typeof(FixedStepSimulationSystemGroup), OrderFirst = true)]
  [BurstCompile]
  public partial struct BasicFixedStepPlayerControlSystem : ISystem
  {
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
      state.RequireForUpdate<FixedTickSystem.Singleton>();
      state.RequireForUpdate(SystemAPI.QueryBuilder().WithAll<BasicPlayer, BasicPlayerInputs>().Build());
    }

    public void OnUpdate(ref SystemState state)
    {
      uint tick = SystemAPI.GetSingleton<FixedTickSystem.Singleton>().Tick;

      foreach (var (playerInputs, player) in SystemAPI.Query<RefRW<BasicPlayerInputs>, BasicPlayer>().WithAll<Simulate>())
      {
        if (SystemAPI.HasComponent<BasicCharacterControl>(player.ControlledCharacter))
        {
          BasicCharacterControl characterControl = SystemAPI.GetComponent<BasicCharacterControl>(player.ControlledCharacter);

          float3 characterUp = MathUtilities.GetUpFromRotation(SystemAPI.GetComponent<LocalTransform>(player.ControlledCharacter).Rotation);

          // Get camera rotation, since our movement is relative to it.
          quaternion cameraRotation = quaternion.identity;
          if (SystemAPI.HasComponent<OrbitCamera>(player.ControlledCamera))
          {
            // Camera rotation is calculated rather than gotten from transform, because this allows us to 
            // reduce the size of the camera ghost state in a netcode prediction context.
            // If not using netcode prediction, we could simply get rotation from transform here instead.
            OrbitCamera orbitCamera = SystemAPI.GetComponent<OrbitCamera>(player.ControlledCamera);
            cameraRotation = OrbitCameraUtilities.CalculateCameraRotation(characterUp, orbitCamera.PlanarForward, orbitCamera.PitchAngle);
          }
          float3 cameraForwardOnUpPlane = math.normalizesafe(MathUtilities.ProjectOnPlane(MathUtilities.GetForwardFromRotation(cameraRotation), characterUp));
          float3 cameraRight = MathUtilities.GetRightFromRotation(cameraRotation);

          // Move
          characterControl.moveVector = (playerInputs.ValueRW.MoveInput.y * cameraForwardOnUpPlane) + (playerInputs.ValueRW.MoveInput.x * cameraRight);
          characterControl.moveVector = MathUtilities.ClampToMaxLength(characterControl.moveVector, 1f);
          characterControl.cameraForward =  cameraForwardOnUpPlane;

          // Jump
          // We detect a jump event if the jump counter has changed since the last fixed update.
          // This is part of a strategy for proper handling of button press events that are consumed during the fixed update group
          characterControl.jump = playerInputs.ValueRW.JumpPressed.IsSet(tick);

          SystemAPI.SetComponent(player.ControlledCharacter, characterControl);
        }
      }
    }
  }
}