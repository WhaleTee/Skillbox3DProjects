using Project.Scripts.KinematicPhysics;
using Unity.CharacterController;
using Unity.Entities;
using Unity.Mathematics;

namespace Project.Scripts.BasicController.Character
{
  public struct BasicCharacterVariableUpdateHandler {
    public KinematicCharacterUpdateContext baseContext;
    public KinematicCharacterData characterData;
    public RefRW<BasicCharacterProperties> characterProperties;
    public RefRW<BasicCharacterControl> characterControl;
  
    public void HandleVariableUpdate() {
      ref KinematicCharacterBody characterBody = ref characterData.characterBody.ValueRW;
      ref quaternion characterRotation = ref characterData.localTransform.ValueRW.Rotation;

      // Add rotation from parent body to the character rotation
      // (this is for allowing a rotating moving platform to rotate your character as well, and handle interpolation properly)
      KinematicCharacterUtilities.AddVariableRateRotationFromFixedRateRotation(
        ref characterRotation,
        characterBody.RotationFromParent,
        baseContext.Time.DeltaTime,
        characterBody.LastPhysicsUpdateDeltaTime
      );

      // Rotate towards move direction
      if (math.lengthsq(characterControl.ValueRO.moveVector) > 0f) {
        CharacterControlUtilities.SlerpRotationTowardsDirectionAroundUp(
          ref characterRotation,
          baseContext.Time.DeltaTime,
          // math.normalizesafe(characterControl.ValueRO.moveVector),
          characterControl.ValueRO.cameraForward,
          MathUtilities.GetUpFromRotation(characterRotation),
          characterProperties.ValueRO.rotationSharpness
        );
      }
    }
  }
}