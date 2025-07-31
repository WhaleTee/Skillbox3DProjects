using Unity.CharacterController;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;

public struct BasicKinematicCharacterPhysicsUpdateProcessor : KinematicCharacterPhysicsUpdateProcessor<KinematicCharacterContext> {
  private readonly RefRW<BasicCharacterProperties> characterProperties;
  private KinematicCharacterData characterData;

  public BasicKinematicCharacterPhysicsUpdateProcessor(RefRW<BasicCharacterProperties> characterProperties, ref KinematicCharacterData characterData) {
    this.characterProperties = characterProperties;
    this.characterData = characterData;
  }

  #region Character Processor Callbacks

  public void UpdateGroundingUp(
    ref KinematicCharacterContext context,
    ref KinematicCharacterUpdateContext baseContext
  ) {
    KinematicCharacterPhysicsMethods.Default_UpdateGroundingUp(ref characterData);
  }

  public bool CanCollideWithHit(
    ref KinematicCharacterContext context,
    ref KinematicCharacterUpdateContext baseContext,
    in BasicHit hit
  ) {
    if (!PhysicsUtilities.IsCollidable(hit.Material)) {
      return false;
    }

    if (PhysicsUtilities.HasPhysicsTag(in baseContext.PhysicsWorld, hit.RigidBodyIndex, characterProperties.ValueRO.ignoreCollisionsTag)) {
      return false;
    }

    return true;
  }

  public bool IsGroundedOnHit(
    ref KinematicCharacterContext context,
    ref KinematicCharacterUpdateContext baseContext,
    in BasicHit hit,
    int groundingEvaluationType
  ) {
    // Ignore grounding
    if (PhysicsUtilities.HasPhysicsTag(in baseContext.PhysicsWorld, hit.RigidBodyIndex, characterProperties.ValueRO.ignoreGroundingTag)) {
      return false;
    }

    // Ignore step handling
    if (characterProperties.ValueRW.stepAndSlopeHandling.StepHandling
        && PhysicsUtilities.HasPhysicsTag(in baseContext.PhysicsWorld, hit.RigidBodyIndex, characterProperties.ValueRO.ignoreStepHandlingTag)) {
      characterProperties.ValueRW.stepAndSlopeHandling.StepHandling = false;
    }

    return KinematicCharacterPhysicsMethods.Default_IsGroundedOnHit(
      in this,
      ref context,
      ref baseContext,
      ref characterData,
      in hit,
      in characterProperties.ValueRW.stepAndSlopeHandling,
      groundingEvaluationType
    );
  }

  public void OnMovementHit(
    ref KinematicCharacterContext context,
    ref KinematicCharacterUpdateContext baseContext,
    ref KinematicCharacterHit hit,
    ref float3 remainingMovementDirection,
    ref float remainingMovementLength,
    float3 originalVelocityDirection,
    float hitDistance
  ) {
    ref var characterPosition = ref characterData.localTransform.ValueRW.Position;

    // Ignore step handling
    if (characterProperties.ValueRW.stepAndSlopeHandling.StepHandling
        && PhysicsUtilities.HasPhysicsTag(in baseContext.PhysicsWorld, hit.RigidBodyIndex, characterProperties.ValueRO.ignoreStepHandlingTag)) {
      characterProperties.ValueRW.stepAndSlopeHandling.StepHandling = false;
    }

    KinematicCharacterPhysicsMethods.Default_OnMovementHit(
      in this,
      ref context,
      ref baseContext,
      ref characterData,
      ref characterPosition,
      ref hit,
      ref remainingMovementDirection,
      ref remainingMovementLength,
      originalVelocityDirection,
      hitDistance,
      characterProperties.ValueRW.stepAndSlopeHandling.StepHandling,
      characterProperties.ValueRW.stepAndSlopeHandling.MaxStepHeight,
      characterProperties.ValueRW.stepAndSlopeHandling.CharacterWidthForStepGroundingCheck
    );
  }

  public void OverrideDynamicHitMasses(
    ref KinematicCharacterContext context,
    ref KinematicCharacterUpdateContext baseContext,
    ref PhysicsMass characterMass,
    ref PhysicsMass otherMass,
    BasicHit hit
  ) {
    if (PhysicsUtilities.HasPhysicsTag(in baseContext.PhysicsWorld, hit.RigidBodyIndex, characterProperties.ValueRO.zeroMassAgainstCharacterTag)) {
      characterMass.InverseMass = 0f;
      characterMass.InverseInertia = new float3(0f);
      otherMass.InverseMass = 1f;
      otherMass.InverseInertia = new float3(1f);
    }

    if (PhysicsUtilities.HasPhysicsTag(in baseContext.PhysicsWorld, hit.RigidBodyIndex, characterProperties.ValueRO.infiniteMassAgainstCharacterTag)) {
      characterMass.InverseMass = 1f;
      characterMass.InverseInertia = new float3(1f);
      otherMass.InverseMass = 0f;
      otherMass.InverseInertia = new float3(0f);
    }
  }

  public void ProjectVelocityOnHits(
    ref KinematicCharacterContext context,
    ref KinematicCharacterUpdateContext baseContext,
    ref float3 velocity,
    ref bool characterIsGrounded,
    ref BasicHit characterGroundHit,
    in DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHits,
    float3 originalVelocityDirection
  ) {
    var latestHit = velocityProjectionHits[^1];

    if (context.bouncySurfaceLookup.HasComponent(latestHit.Entity)) {
      var bouncySurface = context.bouncySurfaceLookup[latestHit.Entity];
      velocity = math.reflect(velocity, latestHit.Normal);
      velocity *= bouncySurface.BounceEnergyMultiplier;
    } else {
      KinematicCharacterPhysicsMethods.Default_ProjectVelocityOnHits(
        ref characterData,
        ref velocity,
        ref characterIsGrounded,
        ref characterGroundHit,
        originalVelocityDirection,
        characterProperties.ValueRO.stepAndSlopeHandling.ConstrainVelocityToGroundPlane
      );
    }
  }

  #endregion
}