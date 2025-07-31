using System.Runtime.CompilerServices;

public static class KinematicCharacterPhysicsExecutor {
  [MethodImpl(MethodImplOptions.AggressiveInlining)]
  public static void Execute(
    KinematicCharacterPhysicsUpdateExecutor physicsUpdateExecutor,
    KinematicCharacterPhysicsUpdateStepHandler physicsUpdateStepsHandler
  ) {
    physicsUpdateExecutor.ExecuteFirstPhysicsStep();
    physicsUpdateStepsHandler.HandlePhysicsIntermediateStep();
    physicsUpdateExecutor.ExecuteSecondPhysicsStep();
  }
}