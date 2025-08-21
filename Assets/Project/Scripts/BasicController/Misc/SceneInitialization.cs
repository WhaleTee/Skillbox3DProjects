using Unity.Entities;

namespace Project.Scripts.BasicController.Misc
{
  [System.Serializable]
  public struct SceneInitialization : IComponentData
  {
    public Entity CharacterSpawnPointEntity;
    public Entity CharacterPrefabEntity;
    public Entity CameraPrefabEntity;
    public Entity PlayerPrefabEntity;
  }
}
