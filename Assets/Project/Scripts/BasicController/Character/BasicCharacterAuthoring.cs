using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using UnityEngine;
using Unity.CharacterController;
using Unity.Physics;
using UnityEngine.Serialization;

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