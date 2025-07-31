using Unity.CharacterController;

public struct BasicKinematicCharacterPhysicsUpdateData {
  public BasicKinematicCharacterPhysicsUpdateProcessor processor;
  public KinematicCharacterContext context;
  public KinematicCharacterUpdateContext baseContext;
  public KinematicCharacterData characterData;
  public BasicCharacterProperties characterProperties;
}