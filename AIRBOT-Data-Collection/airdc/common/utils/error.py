def check_support(key: str, value: str, module: str, supported):
    if value not in supported:
        raise ValueError(
            f"Unsupported {key} '{value}' for {module}. Supported: {supported}"
        )
