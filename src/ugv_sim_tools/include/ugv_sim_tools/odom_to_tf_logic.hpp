#pragma once

#include <cstdint>
#include <optional>

namespace ugv_sim_tools
{

bool shouldPublishOdomTf(std::optional<int64_t> last_pub_stamp_ns, int64_t current_stamp_ns);

}
