using Project.Scripts.KinematicPhysics;
using Unity.CharacterController;

namespace Project.Scripts.BasicController.Character
{
  public struct BasicKinematicCharacterPhysicsUpdateData {
    public BasicKinematicCharacterPhysicsUpdateProcessor processor;
    public KinematicCharacterContext context;
    public KinematicCharacterUpdateContext baseContext;
    public KinematicCharacterData characterData;
    public BasicCharacterProperties characterProperties;
  }
}