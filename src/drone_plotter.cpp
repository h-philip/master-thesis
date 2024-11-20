#include "drone_plotter.h"

dynamic_programming::DronePlotter::DronePlotter(HybridAutomaton* ha, const std::string& directory_name)
  : m_ha(ha), m_out_path(directory_name)
{
  m_ha->addEventListener(this);
  m_noxtime = "set grid; unset xlabel; set xtics border in scale default ('' 0.0";
  m_time = "set grid; set xlabel 'time (s)'; set xtics border in scale default ('0' 0.0";
  init_gnuplot();
}

dynamic_programming::DronePlotter::~DronePlotter()
{
  m_ha->removeEventListener(this);

  flush();

  delete m_gp;
  fclose(m_file);
  delete m_gp_values;
  fclose(m_file_values);
  delete m_gp_path;
  fclose(m_file_path);
  delete m_gp_2dpath;
  fclose(m_file_2dpath);
}

void dynamic_programming::DronePlotter::init_gnuplot()
{
  errno_t err = fopen_s(&m_file, (m_out_path + "plots.plt").c_str(), "w");
  if (err != 0)
  {
    BOOST_LOG_TRIVIAL(error) << "Could not open file " << m_out_path << "plots.plt" << "; code " << err;
    BOOST_LOG_TRIVIAL(warning) << "Gnuplot script will not be written to file. Errors may occur during plotting!";
    m_gp = new Gnuplot("gnuplot -persist");
  }
  else
  {
    m_gp = new Gnuplot(m_file);
    fopen_s(&m_file_values, (m_out_path + "values.plt").c_str(), "w");
    m_gp_values = new Gnuplot(m_file_values);
    fopen_s(&m_file_path, (m_out_path + "path.plt").c_str(), "w");
    m_gp_path = new Gnuplot(m_file_path);
    fopen_s(&m_file_2dpath, (m_out_path + "2dpath.plt").c_str(), "w");
    m_gp_2dpath = new Gnuplot(m_file_2dpath);
  }

  *m_gp << "load 'values.plt'\n";
  *m_gp << "load 'path.plt'\n";
  *m_gp << "load '2dpath.plt'\n";
  m_gp->flush();
  fflush(m_file);
}

void dynamic_programming::DronePlotter::flush()
{
  fseek(m_file_values, 0, SEEK_SET);
  *m_gp_values << "set term qt 0\n";
  plot_values(*m_gp_values);
  m_gp_values->flush();
  fflush(m_file_values);

  fseek(m_file_path, 0, SEEK_SET);
  *m_gp_path << "set term qt 1\n";
  plot_path(*m_gp_path);
  m_gp_path->flush();
  fflush(m_file_path);

  fseek(m_file_2dpath, 0, SEEK_SET);
  *m_gp_2dpath << "set term pngcairo\n";
  *m_gp_2dpath << "set output 'path.png'\n";
  plot_2d_path(*m_gp_2dpath);
  m_gp_2dpath->flush();
  fflush(m_file_2dpath);
}

void dynamic_programming::DronePlotter::on_state_changed(const HybridAutomaton::StateChangedEvent& event)
{
  m_noxtime += ", '' " + std::to_string(event.new_time);
  m_time += ", '" + std::to_string((long)event.new_time) + "' " + std::to_string(event.new_time);
  m_goal_spaces.push_back(StateSpace(event.old_goal_space));
  flush();
}

void dynamic_programming::DronePlotter::on_x_changed(const HybridAutomaton::XChangedEvent& event)
{
  m_disturbance_x.push_back(std::make_pair(event.new_time, event.d.x));
  m_disturbance_y.push_back(std::make_pair(event.new_time, event.d.y));
  m_disturbance_z.push_back(std::make_pair(event.new_time, event.d.z));
  m_accel_x.push_back(std::make_pair(event.new_time, event.u.x));
  m_accel_y.push_back(std::make_pair(event.new_time, event.u.y));
  m_accel_z.push_back(std::make_pair(event.new_time, event.u.z));
  m_vel_x.push_back(std::make_pair(event.new_time, event.new_x[3]));
  m_vel_y.push_back(std::make_pair(event.new_time, event.new_x[4]));
  m_vel_z.push_back(std::make_pair(event.new_time, event.new_x[5]));
  m_coord_x.push_back(std::make_pair(event.new_time, event.new_x[0]));
  m_coord_y.push_back(std::make_pair(event.new_time, event.new_x[1]));
  m_coord_z.push_back(std::make_pair(event.new_time, event.new_x[2]));
  m_coordinates.push_back(boost::make_tuple(event.new_x[0], event.new_x[1], event.new_x[2]));
  if (m_coordinates.size() == 1)
  {
    m_coordinates.push_back(boost::make_tuple(event.new_x[0], event.new_x[1], event.new_x[2]));
  }
}

void dynamic_programming::DronePlotter::plot_values(Gnuplot& gp)
{
  // Macros
  gp << "set macros\n";
  gp << "LMARGIN = 'set lmargin at screen 0.1'\n";
  gp << "NOXTIME = \"" << m_noxtime << ")\"\n";
  gp << "TIME = \"" << m_time << ")\"\n";
  gp << "YTICS = \"\"\n";

  std::string filename;

  // Enable multiplot
  gp << "set multiplot layout 4,1\n";
  // Plot disturbance
  gp << "@LMARGIN; set tmargin at screen 0.98; set bmargin at screen 0.76\n"; // Margins
  gp << "@NOXTIME; @YTICS; set ylabel 'disturbance (m/s^2)'\n"; // Grid, tics, labels
  filename = "disturbance_x.dat";
  gp.file1d(m_disturbance_x, m_out_path + filename);
  gp << "plot '" << filename << "' title 'x disturbance' with lines,"; // x
  filename = "disturbance_y.dat";
  gp.file1d(m_disturbance_y, m_out_path + filename);
  gp << "'" << filename << "' title 'y disturbance' with lines,"; // y
  filename = "disturbance_z.dat";
  gp.file1d(m_disturbance_z, m_out_path + filename);
  gp << "'" << filename << "' title 'z disturbance' with lines\n"; // z
  // Plot acceleration
  gp << "@LMARGIN; set tmargin at screen 0.76; set bmargin at screen 0.54\n"; // Margins
  gp << "@NOXTIME; @YTICS; set ylabel 'acceleration (m/s^2)'\n"; // Grid, tics, labels
  filename = "acceleration_x.dat";
  gp.file1d(m_accel_x, m_out_path + filename);
  gp << "plot '" << filename << "' title 'x acceleration' with lines,"; // x
  filename = "acceleration_y.dat";
  gp.file1d(m_accel_y, m_out_path + filename);
  gp << "'" << filename << "' title 'y acceleration' with lines,"; // y
  filename = "acceleration_z.dat";
  gp.file1d(m_accel_z, m_out_path + filename);
  gp << "'" << filename << "' title 'z acceleration' with lines\n"; // z
  // Plot velocity
  gp << "@LMARGIN; set tmargin at screen 0.54; set bmargin at screen 0.32\n"; // Margins
  gp << "@NOXTIME; @YTICS; set ylabel 'velocity (m/s)'\n"; // Grid, tics, labels
  filename = "velocity_x.dat";
  gp.file1d(m_vel_x, m_out_path + filename);
  gp << "plot '" << filename << "' title 'x velocity' with lines,"; // x
  filename = "velocity_y.dat";
  gp.file1d(m_vel_y, m_out_path + filename);
  gp << "'" << filename << "' title 'y velocity' with lines,"; // y
  filename = "velocity_z.dat";
  gp.file1d(m_vel_z, m_out_path + filename);
  gp << "'" << filename << "' title 'z velocity' with lines\n"; // z
  // Plot coordinates
  gp << "@LMARGIN; set tmargin at screen 0.32; set bmargin at screen 0.1\n"; // Margins
  gp << "@TIME; @YTICS; set ylabel 'coordinate (m)'\n"; // Grid, tics, labels
  filename = "coordinates_x.dat";
  gp.file1d(m_coord_x, m_out_path + filename);
  gp << "plot '" << filename << "' title 'x coordinates' with lines,"; // x
  filename = "coordinates_y.dat";
  gp.file1d(m_coord_y, m_out_path + filename);
  gp << "'" << filename << "' title 'y coordinates' with lines,"; // y
  filename = "coordinates_z.dat";
  gp.file1d(m_coord_z, m_out_path + filename);
  gp << "'" << filename << "' title 'z coordinates' with lines\n"; // z

  gp << "unset multiplot\n";
}

void dynamic_programming::DronePlotter::plot_path(Gnuplot& gp)
{
  CollisionCloud cloud(1, 1, 1, 1);
  cloud.add_collisions_from_file(Config::get_instance().get(Config::Key::COLLISION_CLOUD_FILE), [](unit3 p) {return CollisionCloud::point3((int)p.x, (int)p.y, (int)p.z); });
  std::vector<std::tuple<int, int, int>> obstacles;
  for (auto& p : cloud.get_collisions())
    obstacles.push_back(std::make_tuple(p.x(), p.y(), p.z()));

  // Prepare gnuplot
  gp << "set xlabel 'x (m)'; set ylabel 'y (m)'; set zlabel 'z (m)'\n";
  gp << "set xtics border in scale default autofreq\n";
  gp << "set view equal xyz\n";
  gp << "set view 0, 0\n";
  gp << "set style fill transparent solid .5\n";

  // Plot obstacles
  std::string filename = "obstacles.dat";
  gp.file1d(obstacles, m_out_path + filename);
  gp << "splot '" << filename << "' u 1:2:3:(" << std::to_string(CollisionCloud::MIN_DISTANCE_TO_COLLISION) << ") title 'Obstacles' with circles lc rgb '#2E4057',";

  // Plot goal space cubes
  filename = "goal_space.dat";
  std::ofstream file(m_out_path + filename, std::ios::out);
  for (const StateSpace& space : m_goal_spaces)
  {
    file << space.to_gnuplot_cube() << "\n\n\n";
  }
  file.close();
  gp << "'" << filename << "' u 1:2:3:(1) title 'Goal spaces' with lines lc rgb '#28676d',";

  // Plot route points
  filename = "route.dat";
  std::vector<std::tuple<unit, unit, unit>> route;
  for (const unit3& point : m_ha->get_route())
  {
    route.push_back(std::make_tuple(point.x, point.y, point.z));
  }
  gp.file1d(route, m_out_path + filename);
  gp << "'" << filename << "' title 'Route points' with points pointtype 7 lc rgb '#FF0000',";

  // Plot path
  filename = "path.dat";
  gp.file1d(m_coordinates, m_out_path + filename);
  gp << "'" << filename << "' title 'Path of drone' with lines\n";
}

void dynamic_programming::DronePlotter::plot_2d_path(Gnuplot& gp)
{
  // Get collision cloud and only add points with different x/y values
  CollisionCloud cloud(1, 1, 1, 1);
  cloud.add_collisions_from_file(Config::get_instance().get(Config::Key::COLLISION_CLOUD_FILE), [](unit3 p) {return CollisionCloud::point3((int)p.x, (int)p.y, (int)p.z); });
  std::vector<std::tuple<int, int>> obstacles;
  // Map of x values mapped onto list of y values
  std::unordered_map<int, std::unordered_set<int>> x_to_y;
  for (auto& p : cloud.get_collisions())
  {
    // If x value doesn't exist yet, add a new list with it and also add the y value and the collision to the list
    if (!x_to_y.contains(p.x()))
    {
      x_to_y[p.x()] = std::unordered_set<int>{p.y()};
      obstacles.push_back(std::make_tuple(p.x(), p.y()));
    }
    // Else, check if the y value is already in the list, if not, add it and add the collision to the list
    else if (!x_to_y[p.x()].contains(p.y()))
    {
      x_to_y[p.x()].insert(p.y());
      obstacles.push_back(std::make_tuple(p.x(), p.y()));
    }
  }

  // Prepare gnuplot
  gp << "set xlabel 'x (m)'; set ylabel 'y (m)'\n";
  gp << "set xtics border in scale default autofreq\n";
  gp << "set size ratio -1\n";
  gp << "set key right above\n";
  gp << "set style fill transparent solid .5\n";

  // Prepare goal spaces
  int counter = 1;
  for (const StateSpace& space : m_goal_spaces)
  {
    gp << "set object " << counter++ << " rectangle from " << space.begin[0] << "," << space.begin[1] << " to " << space.end[0] << "," << space.end[1] << " fs empty border lc rgb '#28676d'\n";
  }

  // Plot obstacles
  std::string filename = "obstacles_2d.dat";
  gp.file1d(obstacles, m_out_path + filename);
  gp << "plot '" << filename << "' u 1:2:(" << std::to_string(CollisionCloud::MIN_DISTANCE_TO_COLLISION) << ") title 'Obstacles' with circles lc rgb '#2E4057',";

  // Plot route points
  filename = "route.dat";
  gp << "'" << filename << "' title 'Route points' with points pointtype 7 lc rgb '#FF0000',";

  // Plot goal space cubes
  gp << "NaN title 'Goal spaces' with lines lc rgb '#28676d',";

  // Plot path
  filename = "path.dat";
  gp << "'" << filename << "' title 'Path of drone' with lines\n";
}
