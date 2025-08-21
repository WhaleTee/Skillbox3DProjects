using Unity.Entities;
using UnityEngine;

namespace Project.Scripts.BasicController.Camera
{
  [DisallowMultipleComponent]
  public class MainEntityCameraAuthoring : MonoBehaviour
  {
    public class Baker : Baker<MainEntityCameraAuthoring>
    {
      public override void Bake(MainEntityCameraAuthoring authoring)
      {
        AddComponent<MainEntityCamera>(GetEntity(TransformUsageFlags.Dynamic));
      }
    }
  }
}
