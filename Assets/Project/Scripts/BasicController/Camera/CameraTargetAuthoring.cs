using Unity.Entities;
using UnityEngine;

namespace Project.Scripts.BasicController.Camera
{
  [DisallowMultipleComponent]
  public class CameraTargetAuthoring : MonoBehaviour
  {
    public GameObject Target;

    public class Baker : Baker<CameraTargetAuthoring>
    {
      public override void Bake(CameraTargetAuthoring authoring)
      {
        AddComponent(GetEntity(TransformUsageFlags.Dynamic), new CameraTarget
        {
          TargetEntity = GetEntity(authoring.Target, TransformUsageFlags.Dynamic),
        });
      }
    }
  }
} 