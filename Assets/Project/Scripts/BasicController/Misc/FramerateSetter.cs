using Unity.Entities;
using UnityEngine;

namespace Project.Scripts.BasicController.Misc
{
  public class FramerateSetter : MonoBehaviour
  {
    public int Framerate = -1;
    public float FixedFramerate = 60;
    
    void Start()
    {
      Application.targetFrameRate = Framerate;
        
      FixedStepSimulationSystemGroup fixedSystem = World.DefaultGameObjectInjectionWorld.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>();
      fixedSystem.Timestep = 1f / FixedFramerate;
    }
  }
}
