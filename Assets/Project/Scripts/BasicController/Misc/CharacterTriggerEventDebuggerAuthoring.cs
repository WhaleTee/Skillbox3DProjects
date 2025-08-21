using Unity.Entities;
using UnityEngine;

namespace Project.Scripts.BasicController.Misc
{
  public class CharacterTriggerEventDebuggerAuthoring : MonoBehaviour
  {
    class Baker : Baker<CharacterTriggerEventDebuggerAuthoring>
    {
      public override void Bake(CharacterTriggerEventDebuggerAuthoring authoring)
      {
        AddComponent(GetEntity(authoring, TransformUsageFlags.None), new CharacterTriggerEventDebugger());
      }
    }
  }
}
