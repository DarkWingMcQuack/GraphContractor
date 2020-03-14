#include <algorithm>
#include <future>
#include <vector>

namespace algorithm {

namespace detail {

template<class Iter, class UnaryOperation>
auto transform_par(Iter begin, Iter end, UnaryOperation&& op, std::size_t original_size)
{
    using VectorType = std::vector<decltype(op(*begin))>;

    auto size = std::distance(begin, end);

    if(original_size <= std::thread::hardware_concurrency()
       || size <= (original_size / std::thread::hardware_concurrency())) {
        VectorType ret_vec;
        std::transform(begin,
                       end,
                       std::back_inserter(ret_vec),
                       std::forward<UnaryOperation>(op));
        return ret_vec;
    }

    auto mid = begin + size / 2;

    auto lhsf = std::async(std::launch::async,
                           [&] {
                               return transform_par(begin,
                                                    mid,
                                                    std::forward<UnaryOperation>(op),
                                                    original_size);
                           });

    auto rhsf = std::async(std::launch::async,
                           [&] {
                               return transform_par(mid,
                                                    end,
                                                    std::forward<UnaryOperation>(op),
                                                    original_size);
                           });

    auto lhs = lhsf.get();
    auto rhs = rhsf.get();

    lhs.insert(std::end(lhs),
               std::make_move_iterator(std::begin(rhs)),
               std::make_move_iterator(std::end(rhs)));

    return lhs;
}
} // namespace detail

template<class Iter, class UnaryOperation>
auto transform_par(Iter begin, Iter end, UnaryOperation&& op)
{
    auto original_size = std::distance(begin, end);

    return algorithm::detail::transform_par(begin,
                                            end,
                                            std::forward<UnaryOperation>(op),
                                            original_size);
}

} // namespace algorithm
