using Project.Scripts.KinematicPhysics;
using Unity.CharacterController;
using Unity.Entities;
using Unity.Mathematics;

namespace Project.Scripts.BasicController.Character
{
  public struct BasicKinematicCharacterPhysicsUpdateStepHandler {
    public BasicKinematicCharacterPhysicsUpdateProcessor processor;
    public KinematicCharacterContext context;
    public KinematicCharacterUpdateContext baseContext;
    public KinematicCharacterData characterData;
    public RefRW<BasicCharacterProperties> characterProperties;
    public RefRW<BasicCharacterControl> characterControl;

    public void HandlePhysicsIntermediateStep() {
      var deltaTime = baseContext.Time.DeltaTime;
      ref var characterBody = ref characterData.characterBody.ValueRW;

      // Rotate move input and velocity to take into account parent rotation
      if (characterBody.ParentEntity != Entity.Null) {
        characterControl.ValueRW.moveVector = math.rotate(characterBody.RotationFromParent, characterControl.ValueRO.moveVector);
        characterBody.RelativeVelocity = math.rotate(characterBody.RotationFromParent, characterBody.RelativeVelocity);
      }

      if (characterBody.IsGrounded) {
        // Move on ground
        var targetVelocity = characterControl.ValueRO.moveVector * characterProperties.ValueRO.groundMaxSpeed;

        CharacterControlUtilities.StandardGroundMove_Interpolated(
          ref characterBody.RelativeVelocity,
          targetVelocity,
          characterProperties.ValueRO.groundedMovementSharpness,
          deltaTime,
          characterBody.GroundingUp,
          characterBody.GroundHit.Normal
        );

        // Jump
        if (characterControl.ValueRO.jump) {
          CharacterControlUtilities.StandardJump(
            ref characterBody,
            characterBody.GroundingUp * characterProperties.ValueRO.jumpSpeed,
            true,
            characterBody.GroundingUp
          );
        }

        characterProperties.ValueRW.currentJumpsInAir = 0;
      } else {
        // Move in air
        var airAcceleration = characterControl.ValueRO.moveVector * characterProperties.ValueRO.airAcceleration;

        if (math.lengthsq(airAcceleration) > 0f) {
          var tmpVelocity = characterBody.RelativeVelocity;

          CharacterControlUtilities.StandardAirMove(
            ref characterBody.RelativeVelocity,
            airAcceleration,
            characterProperties.ValueRO.airMaxSpeed,
            characterBody.GroundingUp,
            deltaTime,
            false
          );

          // Cancel air acceleration from input if we would hit a non-grounded surface (prevents air-climbing slopes at high air accelerations)
          if (characterProperties.ValueRO.preventAirAccelerationAgainstUngroundedHits
              && KinematicCharacterPhysicsMethods.MovementWouldHitNonGroundedObstruction(
                processor,
                ref context,
                ref baseContext,
                ref characterData,
                characterBody.RelativeVelocity * deltaTime,
                out var hit
              )) {
            characterBody.RelativeVelocity = tmpVelocity;
          }
        }

        // Jump in air
        if (characterControl.ValueRO.jump && characterProperties.ValueRO.currentJumpsInAir < characterProperties.ValueRO.maxJumpsInAir) {
          CharacterControlUtilities.StandardJump(
            ref characterBody,
            characterBody.GroundingUp * characterProperties.ValueRO.jumpSpeed,
            true,
            characterBody.GroundingUp
          );

          characterProperties.ValueRW.currentJumpsInAir++;
        }

        // Gravity
        CharacterControlUtilities.AccelerateVelocity(ref characterBody.RelativeVelocity, characterProperties.ValueRO.gravity, deltaTime);

        // Drag
        CharacterControlUtilities.ApplyDragToVelocity(ref characterBody.RelativeVelocity, deltaTime, characterProperties.ValueRO.airDrag);
      }
    }
  }
}