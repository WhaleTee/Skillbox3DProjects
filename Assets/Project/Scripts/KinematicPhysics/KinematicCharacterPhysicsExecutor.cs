using System.Runtime.CompilerServices;
using Project.Scripts.BasicController.Character;

namespace Project.Scripts.KinematicPhysics
{
  public static class KinematicCharacterPhysicsExecutor {
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Execute(
      KinematicCharacterPhysicsUpdateExecutor physicsUpdateExecutor,
      BasicKinematicCharacterPhysicsUpdateStepHandler physicsUpdateStepsHandler
    ) {
      physicsUpdateExecutor.ExecuteFirstPhysicsStep();
      physicsUpdateStepsHandler.HandlePhysicsIntermediateStep();
      physicsUpdateExecutor.ExecuteSecondPhysicsStep();
    }
  }
}