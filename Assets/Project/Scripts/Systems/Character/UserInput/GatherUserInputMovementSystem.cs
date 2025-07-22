using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.InputSystem;

[RequireMatchingQueriesForUpdate]
public partial class GatherUserInputMovementSystem : SystemBase
{
    private InputAction moveAction;
    private float2 moveInput;
    
    private InputAction swapMovementModeAction;
    private bool movementMode = true;

    protected override void OnStartRunning()
    {
        moveAction = new InputAction("move", binding: "<Gamepad>/rightStick");
        moveAction.AddCompositeBinding("Dpad")
            .With("Up", binding: "<Keyboard>/w")
            .With("Down", binding: "<Keyboard>/s")
            .With("Left", binding: "<Keyboard>/a")
            .With("Right", binding: "<Keyboard>/d");
        moveAction.started += ctx => moveInput = ctx.ReadValue<Vector2>();
        moveAction.performed += ctx => moveInput = ctx.ReadValue<Vector2>();
        moveAction.canceled += _ => moveInput = float2.zero;
        moveAction.Enable();
        
        swapMovementModeAction = new InputAction("swapMovementMode", binding: "<Keyboard>/g");
        swapMovementModeAction.performed += _ => movementMode = !movementMode;
        swapMovementModeAction.Enable();
    }

    protected override void OnStopRunning()
    {
        moveAction.Disable();
        swapMovementModeAction.Disable();
    }

    protected override void OnUpdate()
    {
        foreach (var (config, entity) in SystemAPI.Query<RefRO<MoveAbilityConfig>>()
                     .WithPresent<MoveAbility>()
                     .WithEntityAccess())
        {
            var speed = movementMode ? config.ValueRO.RunSpeed : config.ValueRO.WalkSpeed;
            var moveAbility = SystemAPI.GetComponent<MoveAbility>(entity);
            moveAbility.Speed = speed;
            moveAbility.Direction = new float3(moveInput.x, 0, moveInput.y);
            SystemAPI.SetComponent(entity, moveAbility);
            SystemAPI.SetComponentEnabled<MoveAbility>(entity, !moveInput.Equals(float2.zero));
        }
    }
}