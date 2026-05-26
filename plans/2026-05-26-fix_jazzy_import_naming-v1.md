# Fix ROS 2 Jazzy Service Request/Response Import Naming

## Objective

Fix the `ImportError: cannot import name 'ListControllersRequest' from 'controller_manager_msgs.srv'` crash in `force_controller_node`. In ROS 2 Jazzy, `controller_manager_msgs` renamed service request/response classes from `CamelCase` (e.g., `ListControllersRequest`) to `Snake_Case` with underscores (e.g., `ListControllers_Request`). The codebase currently imports the old Humble-era names, which no longer exist.

## Root Cause

Two files import deprecated request/response class names that were removed in ROS 2 Jazzy:

1. `src/force_controller/force_controller/controller_manager_client.py:30-38` — imports 6 old-style names and uses them throughout the file (lines 32, 33, 35, 37, 52, 53, 99, 114, 171).
2. `src/force_controller/test/test_controller_manager_client.py:39-47` — mocks the same 6 old-style names.

The Jazzy naming convention is `ServiceName_Request` / `ServiceName_Response` (underscore separator) instead of `ServiceNameRequest` / `ServiceNameResponse` (no separator).

## Implementation Plan

- [x] **Task 1. Update imports in `controller_manager_client.py`** (lines 30-38)
  - Replace `ListControllersRequest` → `ListControllers_Request`
  - Replace `ListControllersResponse` → `ListControllers_Response`  
  - Replace `LoadControllerRequest` → `LoadController_Request`
  - Replace `SwitchControllerRequest` → `SwitchController_Request`
  - Remove unused imports (`ListControllersResponse` is imported but never directly referenced — only used implicitly via the service call return). Keep if needed for type hints; remove if purely unused.
  - Rationale: These are the exact names the error message points to. The ROS 2 Jazzy `controller_manager_msgs` package only exports the underscore-separated variants.

- [x] **Task 2. Update class attribute references in `controller_manager_client.py`** (lines 52-53)
  - `SwitchControllerRequest.STRICT` → `SwitchController_Request.STRICT`
  - `SwitchControllerRequest.BEST_EFFORT` → `SwitchController_Request.BEST_EFFORT`
  - Rationale: The class reference must match the new import name.

- [x] **Task 3. Update request instantiation in `controller_manager_client.py`** (lines 99, 114, 171)
  - `ListControllersRequest()` → `ListControllers_Request()` (line 99)
  - `LoadControllerRequest()` → `LoadController_Request()` (line 114)
  - `SwitchControllerRequest()` → `SwitchController_Request()` (line 171)
  - Rationale: Constructor calls must use the renamed classes.

- [x] **Task 4. Update mock registrations in `test_controller_manager_client.py`** (lines 39-47)
  - `mock_controller_manager_msgs_srv.ListControllersRequest` → `mock_controller_manager_msgs_srv.ListControllers_Request`
  - `mock_controller_manager_msgs_srv.ListControllersResponse` → `mock_controller_manager_msgs_srv.ListControllers_Response`
  - `mock_controller_manager_msgs_srv.LoadControllerRequest` → `mock_controller_manager_msgs_srv.LoadController_Request`
  - `mock_controller_manager_msgs_srv.LoadControllerResponse` → `mock_controller_manager_msgs_srv.LoadController_Response`
  - `mock_controller_manager_msgs_srv.SwitchControllerRequest` → `mock_controller_manager_msgs_srv.SwitchController_Request`
  - `mock_controller_manager_msgs_srv.SwitchControllerResponse` → `mock_controller_manager_msgs_srv.SwitchController_Response`
  - Rationale: Tests must mock the same names the production code imports, otherwise the mock patching is ineffective.

- [x] **Task 5. Verify no other files reference the old-style names**
  - Search the entire `src/` tree for any remaining `ListControllersRequest`, `LoadControllerRequest`, `SwitchControllerRequest` (and corresponding `Response` variants).
  - Rationale: The `fs_search` results confirm only the two files above contain these references, but a final verification ensures completeness.

## Verification Criteria

- [ ] `force_controller_node` starts without `ImportError` when launched in the Docker/ROS 2 Jazzy environment
- [ ] `python3 -c "from controller_manager_msgs.srv import ListControllers_Request, LoadController_Request, SwitchController_Request"` succeeds in the container
- [ ] Unit tests in `test/test_controller_manager_client.py` pass with the updated mock names
- [ ] No remaining references to old-style (`CamelCase`) request/response names exist in the codebase

## Potential Risks and Mitigations

1. **Other packages may also use old-style names**
   Mitigation: The `fs_search` across all of `src/` confirmed only `force_controller` is affected. No other package imports these names.

2. **`SwitchController_Request.STRICT` / `.BEST_EFFORT` constants may have been removed or renamed in Jazzy**
   Mitigation: These are class-level constants on the request message class and are part of the `SwitchController.srv` definition. They should still exist under the renamed class. If not, use literal integer values (2 and 1) as fallback.

3. **The `LoadControllerResponse` import was unused in production code but may be needed for type annotations**
   Mitigation: If it was truly unused, removing it is safe. If type checkers flag it, add `LoadController_Response` back.

## Alternative Approaches

1. **Compatibility shim**: Create a small compatibility module that tries the new names first and falls back to old names (for Humble support). This adds complexity for no current benefit since the project targets Jazzy.
2. **Use `getattr()` with fallback**: Dynamically resolve names at runtime. This obscures the actual imports and makes static analysis harder — not recommended.
3. **Direct literal constants**: Replace `SwitchController_Request.STRICT` with `2` and `SwitchController_Request.BEST_EFFORT` with `1`. Simpler but loses self-documenting semantics.

**Recommended approach**: Straightforward rename (Tasks 1-4). It's the cleanest, most maintainable solution.
