using Unity.Entities;
using UnityEngine;

namespace Project.Scripts.BasicController.Misc
{
  public class SelfDestructAfterTimeAuthoring : MonoBehaviour
  {
    public float LifeTime = 1f;

    public class Baker : Baker<SelfDestructAfterTimeAuthoring>
    {
      public override void Bake(SelfDestructAfterTimeAuthoring authoring)
      {
        Entity entity = GetEntity(TransformUsageFlags.None);
        AddComponent(entity, new SelfDestructAfterTime
        {
          LifeTime = authoring.LifeTime,
        });
      }
    }
  }
}