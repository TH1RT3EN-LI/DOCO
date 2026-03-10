#include "ugv_sim_tools/odom_to_tf_logic.hpp"

namespace ugv_sim_tools
{

bool shouldPublishOdomTf(std::optional<int64_t> last_pub_stamp_ns, int64_t current_stamp_ns)
{
  return !last_pub_stamp_ns.has_value() || current_stamp_ns > *last_pub_stamp_ns;
}

}
