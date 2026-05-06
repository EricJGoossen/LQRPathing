#ifndef TOPOLOGICAL_SORT_HPP_
#define TOPOLOGICAL_SORT_HPP_

#include <array>
#include <tuple>
#include <utility>
#include <vector>

namespace auto_nav {
class TopologicalSort {
  public:
    template <typename... Types>
    struct TypeList {};

    template <typename... Types>
    using DependenciesList = std::tuple<Types...>;

    template <typename... Types>
    static constexpr auto Sort() {
        constexpr MaxValues max_values = FindMaxValues<Types...>();
        constexpr int total_ids = max_values.max_id + 1;
        constexpr int max_num_deps = max_values.max_num_deps;
        constexpr int num_unique = FindNumUnique<total_ids, Types...>();

        return UnpackTypeMappings<
            SortMappings<num_unique, max_num_deps>(
                CreateUniqueArray<num_unique, max_num_deps, Types...>()
            ),
            Types...>();
    }

    template <typename... Types>
    struct SortResult {
      private:
        static constexpr auto kSorted = TopologicalSort::Sort<Types...>();
        using Sorted = std::remove_const_t<decltype(kSorted)>;

      public:
        template <typename Base>
        static std::vector<Base*> CreateInstancesFromTypeList() {
            return CreateInstancesImpl<Base, Sorted>();
        }

      private:
        template <typename Base, typename TypeList>
        struct Impl;

        template <typename Base, typename... SortedTypes>
        struct Impl<Base, TypeList<SortedTypes...>> {
            static std::vector<Base*> run() { return {new SortedTypes{}...}; }
        };

        template <typename Base, typename TypeList>
        static auto CreateInstancesImpl() {
            return Impl<Base, TypeList>::run();
        }
    };

  private:
    static constexpr int INVALID_ID = -1;

    struct MaxValues {
        int max_id;
        int max_num_deps;
    };

    struct TypeMapping {
        int idx;
        int count = 0;
    };

    template <int MaxNumDeps>
    struct Node {
        int id = 0;
        int in_degree = 0;
        TypeMapping type_mapping{};
        std::array<int, MaxNumDeps> dependencies{};
    };

    template <typename... Types>
    static constexpr MaxValues FindMaxValues() {
        MaxValues max_values{0, 0};
        auto find_max_values = [&max_values]<typename T>() {
            if (T::ID > max_values.max_id) {
                max_values.max_id = T::ID;
            }

            using Dependencies = typename T::Dependencies;
            int num_deps = std::tuple_size_v<Dependencies>;
            if (num_deps > max_values.max_num_deps) {
                max_values.max_num_deps = num_deps;
            }
        };

        (..., find_max_values.template operator()<Types>());
        return max_values;
    }

    template <int ListSize, typename... Types>
    static constexpr int FindNumUnique() {
        int num_unique = 0;
        std::array<bool, ListSize> seen_types{};
        auto count_unique_types = [&num_unique, &seen_types]<typename T>() {
            if (!seen_types[T::ID]) {
                seen_types[T::ID] = true;
                num_unique++;
            }
        };

        (..., count_unique_types.template operator()<Types>());
        return num_unique;
    }

    template <int NumUnique, int MaxNumDeps, typename... Types>
    static constexpr std::array<Node<MaxNumDeps>, NumUnique> CreateUniqueArray(
    ) {
        std::array<Node<MaxNumDeps>, NumUnique> unique_types{};
        int unique_idx = 0;

        auto make_generator_info = [&unique_idx, &unique_types]<typename T>() {
            for (int i = 0; i < unique_idx; ++i) {
                if (unique_types[i].id == T::ID) {
                    unique_types[i].type_mapping.count++;
                    return;
                }
            }

            Node<MaxNumDeps> node{};
            node.id = T::ID;
            node.type_mapping = TypeMapping{unique_idx, 1};
            node.dependencies.fill(INVALID_ID);

            unique_types[unique_idx] = node;
            unique_idx++;

            auto& deps = unique_types[unique_idx - 1].dependencies;
            using Dependencies = typename T::Dependencies;

            [&deps]<
                typename... DependencyTypes>(std::tuple<DependencyTypes...>) {
                static_assert(sizeof...(DependencyTypes) <= MaxNumDeps);
                int dep_idx = 0;
                (..., (deps[dep_idx++] = DependencyTypes::ID));
            }(Dependencies{});
        };

        (..., make_generator_info.template operator()<Types>());

        for (Node<MaxNumDeps>& node : unique_types) {
            for (int dep_id : node.dependencies) {
                if (dep_id == INVALID_ID) break;
                node.in_degree++;
            }
        }

        return unique_types;
    }

    template <typename T>
    struct TupleToTypeList;

    template <typename... Types>
    struct TupleToTypeList<std::tuple<Types...>> {
        using type = TypeList<Types...>;
    };

    template <auto Arr, typename... Types>
    static constexpr auto UnpackTypeMappings() {
        return []<std::size_t... I>(std::index_sequence<I...>) {
            auto repeat_type =
                []<typename T, std::size_t... J>(std::index_sequence<J...>) {
                    return std::tuple<std::conditional_t<
                        true,
                        T,
                        std::integral_constant<std::size_t, J>>...>{};
                };

            return typename TupleToTypeList<
                decltype(std::tuple_cat(repeat_type.template operator(
                )<std::tuple_element_t<Arr[I].idx, std::tuple<Types...>>>(
                    std::make_index_sequence<Arr[I].count>{}
                )...))>::type{};
        }
        (std::make_index_sequence<std::size(Arr)>{});
    }

    template <int NumUnique, int MaxNumDeps>
    static constexpr std::array<TypeMapping, NumUnique> SortMappings(
        std::array<Node<MaxNumDeps>, NumUnique> unique_mappings
    ) {
        int front = 0, back = 0;
        std::array<int, NumUnique> queue{};

        for (int i = 0; i < NumUnique; ++i) {
            if (unique_mappings[i].in_degree == 0) {
                queue[back++] = i;
            }
        }

        std::array<TypeMapping, NumUnique> final_order_indicies{};
        int idx = 0;

        while (front < back) {
            int node = queue[front++];
            final_order_indicies[idx++] = unique_mappings[node].type_mapping;

            for (int j = 0; j < NumUnique; ++j) {
                for (int i = 0; i < MaxNumDeps; ++i) {
                    if (unique_mappings[j].dependencies[i] == INVALID_ID) break;

                    if (unique_mappings[j].dependencies[i] ==
                        unique_mappings[node].id) {
                        if (--unique_mappings[j].in_degree == 0) {
                            queue[back++] = j;
                        }
                        break;
                    }
                }
            }
        }

        if (idx != NumUnique) {
            throw "Cycle detected in dependencies";
        }

        return final_order_indicies;
    }
};
}  // namespace auto_nav

#endif  // TOPOLOGICAL_SORT_HPP_