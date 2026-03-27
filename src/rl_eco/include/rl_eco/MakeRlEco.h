#pragma once

namespace ord {
  class OpenRoad;
}

namespace eco {
  class RlEco;
}

namespace ord {
class OpenRoad;
  eco::RlEco* getRlEco();
  void initRlEco(OpenRoad* openroad);
  void deleteRlEco(eco::RlEco* eco);
} // namespace ord
