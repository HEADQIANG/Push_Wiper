from pydantic import BaseModel
from typing import Set, Dict, Any, TypeVar
from functools import cache, cached_property


T = TypeVar("T", bound=Dict[str, Any])


class DictKeyFilterConfig(BaseModel, frozen=True):
    """Config for DictKeyFilter."""

    include_equal: Set[str] = set()
    include_endswith: Set[str] = set()
    include_fragments: Set[str] = set()
    exclude_keys: Set[str] = set()
    exclude_endswith: Set[str] = set()
    exclude_fragments: Set[str] = set()

    def __bool__(self):
        return bool(
            self.include_equal
            or self.include_endswith
            or self.include_fragments
            or self.exclude_keys
            or self.exclude_endswith
            or self.exclude_fragments
        )

    @cached_property
    def include_all(self) -> bool:
        return not (
            self.include_equal or self.include_endswith or self.include_fragments
        )


class DictKeyFilter:
    def __init__(self, config: DictKeyFilterConfig):
        self.config = config
        self._filter_flag = bool(config)

    def __call__(self, data: T) -> T:
        if self._filter_flag:
            result = {}
            for key, value in data.items():
                if not self._should_include(key):
                    continue
                if self._should_exclude(key):
                    continue
                result[key] = value
            return result
        return data

    @cache
    def _should_include(self, key: str) -> bool:
        if self.config.include_all:
            return True
        if key in self.config.include_equal:
            return True
        if any(key.endswith(suffix) for suffix in self.config.include_endswith):
            return True
        if any(fragment in key for fragment in self.config.include_fragments):
            return True
        return False

    @cache
    def _should_exclude(self, key: str) -> bool:
        if key in self.config.exclude_keys:
            return True
        if any(key.endswith(suffix) for suffix in self.config.exclude_endswith):
            return True
        if any(fragment in key for fragment in self.config.exclude_fragments):
            return True
        return False


# 使用示例
if __name__ == "__main__":
    # 测试数据
    test_dict = {
        "user_name": "Alice",
        "user_age": 30,
        "user_email": "alice@example.com",
        "admin_role": "superuser",
        "temp_token": "abc123",
        "session_id": "xyz789",
        "user_id": 1001,
        "password": "secret",
    }

    # 示例1: 只包含以 "user_" 开头的键
    config1 = DictKeyFilterConfig(include_fragments={"user_"})
    filter1 = DictKeyFilter(config1)
    print("示例1 - 只包含 user_ 相关:")
    print(filter1(test_dict))
    # 输出: {'user_name': 'Alice', 'user_age': 30, 'user_email': 'alice@example.com', 'user_id': 1001}

    # 示例2: 包含 user_ 相关，但排除 password 和以 _id 结尾的
    config2 = DictKeyFilterConfig(
        include_fragments={"user_"}, exclude_keys={"password"}, exclude_endswith={"_id"}
    )
    filter2 = DictKeyFilter(config2)
    print("\n示例2 - 包含 user_ 但排除 password 和 _id 结尾:")
    print(filter2(test_dict))
    # 输出: {'user_name': 'Alice', 'user_age': 30, 'user_email': 'alice@example.com'}

    # 示例3: 不设置 include，只排除敏感信息
    config3 = DictKeyFilterConfig(
        exclude_keys={"password"}, exclude_fragments={"temp_", "session_"}
    )
    filter3 = DictKeyFilter(config3)
    print("\n示例3 - 排除敏感信息:")
    print(filter3(test_dict))
    # 输出: {'user_name': 'Alice', 'user_age': 30, 'user_email': 'alice@example.com',
    #        'admin_role': 'superuser', 'user_id': 1001}

    # 示例4: 精确匹配包含
    config4 = DictKeyFilterConfig(include_equal={"user_name", "user_age"})
    filter4 = DictKeyFilter(config4)
    print("\n示例4 - 精确匹配:")
    print(filter4(test_dict))
    # 输出: {'user_name': 'Alice', 'user_age': 30}
