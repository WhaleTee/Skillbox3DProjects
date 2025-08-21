using Project.Scripts.BasicController.Character;
using Unity.CharacterController;
using Unity.Mathematics;

namespace Project.Scripts.KinematicPhysics
{
  public struct KinematicCharacterPhysicsUpdateExecutor {
    public BasicKinematicCharacterPhysicsUpdateProcessor physicsUpdateProcessor;
    public KinematicCharacterContext context;
    public KinematicCharacterUpdateContext baseContext;
    public KinematicCharacterData characterData;
    public BasicStepAndSlopeHandlingParameters stepAndSlopeParameters;
    public float3 gravity;
  
    public void ExecuteFirstPhysicsStep() {
      KinematicCharacterPhysicsMethods.Update_Initialize(
        in physicsUpdateProcessor,
        ref context,
        ref baseContext,
        ref characterData,
        baseContext.Time.DeltaTime
      );

      KinematicCharacterPhysicsMethods.Update_ParentMovement(
        in physicsUpdateProcessor,
        ref context,
        ref baseContext,
        ref characterData,
        ref characterData.localTransform.ValueRW.Position,
        characterData.characterBody.ValueRO.WasGroundedBeforeCharacterUpdate
      );

      KinematicCharacterPhysicsMethods.Update_Grounding(
        in physicsUpdateProcessor,
        ref context,
        ref baseContext,
        ref characterData,
        ref characterData.localTransform.ValueRW.Position
      );
    }

    public void ExecuteSecondPhysicsStep() {
      KinematicCharacterPhysicsMethods.Update_PreventGroundingFromFutureSlopeChange(
        in physicsUpdateProcessor,
        ref context,
        ref baseContext,
        ref characterData,
        in stepAndSlopeParameters
      );

      KinematicCharacterPhysicsMethods.Update_GroundPushing(in physicsUpdateProcessor, ref context, ref baseContext, ref characterData, gravity);

      KinematicCharacterPhysicsMethods.Update_MovementAndDecollisions(
        in physicsUpdateProcessor,
        ref context,
        ref baseContext,
        ref characterData,
        ref characterData.localTransform.ValueRW.Position
      );

      KinematicCharacterPhysicsMethods.Update_MovingPlatformDetection(ref baseContext, ref characterData.characterBody.ValueRW);
      KinematicCharacterPhysicsMethods.Update_ParentMomentum(ref baseContext, ref characterData);
      KinematicCharacterPhysicsMethods.Update_ProcessStatefulCharacterHits(ref characterData);
    }
  }
}