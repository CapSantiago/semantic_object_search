// Use Fortress's own mesh loader so COLLADA units/up-axis match simulation.
#include <ignition/common/Mesh.hh>
#include <ignition/common/MeshManager.hh>
#include <iostream>
#include <iomanip>
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  const auto *mesh = ignition::common::MeshManager::Instance()->Load(argv[1]);
  if (!mesh) return 1;
  const auto lo = mesh->Min(), hi = mesh->Max();
  std::cout << std::setprecision(12) << "[" << lo.X() << "," << lo.Y() << ","
            << lo.Z() << "," << hi.X() << "," << hi.Y() << "," << hi.Z() << "]\n";
}
