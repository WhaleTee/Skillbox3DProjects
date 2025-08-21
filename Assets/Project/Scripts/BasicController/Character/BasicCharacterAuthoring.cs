using Unity.CharacterController;
using Unity.Entities;
using UnityEngine;

namespace Project.Scripts.BasicController.Character
{
  [DisallowMultipleComponent]
  public class BasicCharacterAuthoring : MonoBehaviour
  {
    public AuthoringKinematicCharacterProperties characterProperties = AuthoringKinematicCharacterProperties.GetDefault();
    public BasicCharacterProperties character = BasicCharacterProperties.GetDefault();

    public class Baker : Baker<BasicCharacterAuthoring>
    {
      public override void Bake(BasicCharacterAuthoring authoring)
      {
        KinematicCharacterUtilities.BakeCharacter(this, authoring, authoring.characterProperties);

        AddComponent(GetEntity(TransformUsageFlags.Dynamic), authoring.character);
        AddComponent(GetEntity(TransformUsageFlags.Dynamic), new BasicCharacterControl());
      }
    }
  }
}