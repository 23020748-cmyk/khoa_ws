"""Strict whitelist and workflow validation for LLM-generated skill plans."""


class PlanValidationError(ValueError):
    """Raised when a plan contains an unsupported or unsafe instruction."""


class TaskValidator:
    SKILLS = {"pick", "place", "home"}
    MAX_STEPS = 32

    def __init__(self, objects, zones):
        self.objects = set(objects)
        self.zones = set(zones)

    def validate(self, payload):
        if not isinstance(payload, dict) or set(payload) != {"plan"}:
            raise PlanValidationError("Expected a JSON object with exactly one 'plan' field.")
        steps = payload["plan"]
        if not isinstance(steps, list) or not steps or len(steps) > self.MAX_STEPS:
            raise PlanValidationError(f"'plan' must contain 1 to {self.MAX_STEPS} steps.")

        held = None
        picked = set()
        placed = set()
        validated = []
        for index, step in enumerate(steps):
            if not isinstance(step, dict) or not isinstance(step.get("skill"), str):
                raise PlanValidationError(f"Step {index + 1} must contain a skill string.")
            skill = step["skill"].strip().lower()
            if skill not in self.SKILLS:
                raise PlanValidationError(f"Unsupported skill at step {index + 1}: {step['skill']!r}.")
            if skill == "pick":
                if set(step) != {"skill", "object"}:
                    raise PlanValidationError(f"pick at step {index + 1} requires only 'object'.")
                obj = self._check_object(step["object"], index)
                if held is not None:
                    raise PlanValidationError("Place the held object before picking another.")
                if obj in picked:
                    raise PlanValidationError(f"Object {obj!r} is picked more than once.")
                held = obj
                picked.add(obj)
                validated.append({"skill": skill, "object": obj})
            elif skill == "place":
                if set(step) != {"skill", "object", "zone"}:
                    raise PlanValidationError(f"place at step {index + 1} requires 'object' and 'zone'.")
                obj = self._check_object(step["object"], index)
                zone = step["zone"]
                if not isinstance(zone, str) or zone not in self.zones:
                    raise PlanValidationError(f"Invalid zone at step {index + 1}: {zone!r}.")
                if held != obj:
                    raise PlanValidationError(f"Cannot place {obj!r}; it is not currently held.")
                held = None
                placed.add(obj)
                validated.append({"skill": skill, "object": obj, "zone": zone})
            else:
                if set(step) != {"skill"}:
                    raise PlanValidationError(f"home at step {index + 1} takes no parameters.")
                if index != len(steps) - 1:
                    raise PlanValidationError("home() must be the final step.")
                if held is not None:
                    raise PlanValidationError("The plan cannot finish with an object still held.")
                validated.append({"skill": skill})

        if validated[-1]["skill"] != "home":
            raise PlanValidationError("Every task plan must finish with home().")
        if picked != placed:
            raise PlanValidationError("Every picked object must have one matching place step.")
        return validated

    def _check_object(self, obj, index):
        if not isinstance(obj, str) or obj not in self.objects:
            raise PlanValidationError(f"Invalid object at step {index + 1}: {obj!r}.")
        return obj
