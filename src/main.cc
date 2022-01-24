// Copyright 2019 ETH Zürich, Thomas Schöps
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This program shortly demonstrates the use of the generic camera models by
// example. It runs several functions that act as unit tests.

#include <iostream>

#include "central_generic.h"
#include "noncentral_generic.h"
#include "opencv2/opencv.hpp"
using namespace cv;

bool TestCentralGenericCameraReprojection() {
  // Initialize a camera arbitrarily for testing
  CentralGenericCamera<double> camera(
      /*width*/ 640,
      /*height*/ 480,
      /*calibration_min_x*/ 0,
      /*calibration_min_y*/ 0,
      /*calibration_max_x*/ 639,
      /*calibration_max_y*/ 479,
      /*grid_width*/ 8,
      /*grid_height*/ 8);
  for (int y = 0; y < camera.grid_height(); ++ y) {
    for (int x = 0; x < camera.grid_width(); ++ x) {
      camera.grid_value(x, y) = Eigen::Vector3d(x, y, 1).normalized();
    }
  }
  
  // Verify that un-projecting a pixel and projecting the result again returns
  // the original pixel coordinate.
  int step = 10;
  for (int y = 0; y < camera.height(); y += step) {
    for (int x = 0; x < camera.width(); x += step) {
      // Note that 0.5 has to be added to x and y to get the center of the pixel.
      Eigen::Vector2d pixel(x + 0.5, y + 0.5);
      
      Eigen::Vector3d direction;
      if (!camera.Unproject(pixel, &direction)) {
        std::cout << "Unprojection failed! Pixel: (" << pixel.transpose() << ")" << std::endl;
        return false;
      }
      
      Eigen::Vector2d reprojected_pixel;
      if (!camera.Project(direction, &reprojected_pixel)) {
        std::cout << "Reprojection failed! Pixel: (" << pixel.transpose() << "), Direction: (" << direction.transpose() << ")" << std::endl;
        return false;
      }
      
      if ((pixel - reprojected_pixel).norm() > 1e-3) {
        std::cout << "Reprojection gave an incorrect result! Pixel: (" << pixel.transpose() << "), Reprojected pixel: (" << reprojected_pixel.transpose() << ")" << std::endl;
        return false;
      }
    }
  }
  
  return true;
}


bool TestCentralGenericCameraRead() {
  const char* yaml_text = R"yaml(type : CentralGenericModel
width : 640
height : 480
calibration_min_x : 15
calibration_min_y : 16
calibration_max_x : 624
calibration_max_y : 464
grid_width : 17
grid_height : 13
# The grid is stored in row-major order, top to bottom. Each row is stored left to right. Each grid point is stored as x, y, z.
grid : [-0.53894559551046, -0.44771088301023, 0.71350726016914, -0.50746422155752, -0.44379640008213, 0.73859651983554, -0.44437619310142, -0.45781771425039, 0.7700212591372, -0.37997210231294, -0.46870315339742, 0.79745755715223, -0.31059196744565, -0.47963002013777, 0.82066294758621, -0.23686194190801, -0.48914671224284, 0.83942356077107, -0.15994227346032, -0.49649135970636, 0.85317923023083, -0.081933919994339, -0.50282605963066, 0.86049566327243, 0.00017436903287285, -0.50517167333542, 0.86301885845845, 0.080672748404693, -0.50583327067508, 0.85885074951529, 0.16154830952315, -0.50237584880524, 0.849423716544, 0.23896441104945, -0.49642692582469, 0.83453958418282, 0.31402633020527, -0.48955063361265, 0.81346643512026, 0.3843069083144, -0.47937130762018, 0.78899388442012, 0.45079034867314, -0.46992632280404, 0.75891851517736, 0.51358957917605, -0.45797921215404, 0.72558995679139, 0.56184318840458, -0.44952932685694, 0.69444626569586, -0.57546755405715, -0.3672276897273, 0.73073997983211, -0.51677353635779, -0.37947756936268, 0.76742549245569, -0.45558092006952, -0.39047839567188, 0.79998290468118, -0.38970434913988, -0.40134679499028, 0.82888555929784, -0.31928895242758, -0.41148306432781, 0.85366050197317, -0.24471821619376, -0.42086031485541, 0.87349275328577, -0.1660594173112, -0.42874617551075, 0.8880320866428, -0.08381796849411, -0.43440908170052, 0.89680728024121, -1.7081254330203e-05, -0.43757902694522, 0.89917995689734, 0.084565424667482, -0.43740623721691, 0.89527899148509, 0.16771238238175, -0.43437477731126, 0.8849808526921, 0.24816249429, -0.42882906468793, 0.86863168817781, 0.32465769408562, -0.4210171713848, 0.84696040230352, 0.39673122698072, -0.41221831671241, 0.82017095346346, 0.46402890401374, -0.40204023074769, 0.78932935400893, 0.52614171369688, -0.39200672375951, 0.75465596508306, 0.58573098117777, -0.37961077757712, 0.71611093779932, -0.58337952619307, -0.30024225891515, 0.75466741971565, -0.5269686950245, -0.30932446373534, 0.79159482729425, -0.4653726891669, -0.31910988982622, 0.82558896455359, -0.39915751434761, -0.32878091988209, 0.85590676213086, -0.32815008434915, -0.33834268088517, 0.88195337316271, -0.25223771497102, -0.34719446558684, 0.90323426541073, -0.17160702269894, -0.35506116012136, 0.91895734521994, -0.087160015687684, -0.36109050029189, 0.92844858891824, 0.00022196357964258, -0.36427314514739, 0.93129212734598, 0.088106920804705, -0.36429666797063, 0.92710576970042, 0.17433073279732, -0.36091396242951, 0.91616041571647, 0.25716160308463, -0.35505754660395, 0.89877808634753, 0.33518264752635, -0.34763985541736, 0.87567067081331, 0.40821864626997, -0.33897921582614, 0.84761466957304, 0.47594022843835, -0.33011336610108, 0.81517241395637, 0.53872042431996, -0.32052128490729, 0.77913183116973, 0.59604117688413, -0.31203246844617, 0.73984502032113, -0.59285401222701, -0.22762180153511, 0.77247164067831, -0.53547534092657, -0.2353922744755, 0.81108361860959, -0.47399108295228, -0.24344494855175, 0.84620742747057, -0.40758925910656, -0.25155154429551, 0.87783416225589, -0.3362264820007, -0.25972430800777, 0.90525965149857, -0.25961756314713, -0.2678629856886, 0.92781902427334, -0.17763264723781, -0.27524102318751, 0.94482221702814, -0.090301835218613, -0.28107164228047, 0.95542886206243, 0.00019178390817439, -0.28445813903464, 0.95868844279874, 0.091680166942471, -0.28442566983458, 0.95430434627982, 0.18111525713032, -0.28110039371353, 0.94243293251494, 0.2658302066706, -0.27539853569176, 0.92384519685946, 0.34514834670517, -0.2683058398658, 0.89938011711433, 0.41860826876897, -0.26086241363991, 0.86989534914736, 0.48660593673988, -0.25303786372708, 0.83617372707467, 0.54919096238027, -0.24556892414285, 0.79880234747724, 0.60744128841104, -0.23775382152132, 0.75794999933079, -0.59904701821697, -0.15268001638937, 0.78602257127941, -0.54211283204213, -0.15839159844818, 0.82524286054245, -0.48072470943623, -0.16405685777881, 0.86138789238832, -0.41446200216416, -0.17000632205711, 0.89404647486732, -0.34310499731614, -0.17625957543388, 0.92261125230758, -0.26613314424955, -0.1826108585575, 0.94648107422639, -0.18274548937198, -0.188774502309, 0.96486697186306, -0.093745511055916, -0.19397368341046, 0.97651727547496, 5.5019588706514e-05, -0.19710856524831, 0.98038166571932, 0.095044124875147, -0.19712330995603, 0.97576073655312, 0.18726740652064, -0.19424381754086, 0.96291238324273, 0.27384409289581, -0.18935608155699, 0.94294946161682, 0.35369195202029, -0.18367901678986, 0.9171499451383, 0.4272768077013, -0.1777731917287, 0.88647121888033, 0.49516677031978, -0.17216171138678, 0.85156926594579, 0.55754631058517, -0.16653104165948, 0.81327087966843, 0.61522632822527, -0.16154760006191, 0.77162098077533, -0.60332131938421, -0.076151526209203, 0.79385409908402, -0.54632732276171, -0.078826935541378, 0.83385416628879, -0.4851648784719, -0.081873812418588, 0.87058125383987, -0.41905670488412, -0.085111816613577, 0.90396208812339, -0.34785360241488, -0.088543062794228, 0.9333584506062, -0.27070592190367, -0.092246448594337, 0.95823217258033, -0.18713721814225, -0.096041745827401, 0.9776275592701, -0.096284751947301, -0.099393729319453, 0.99037878264552, -0.00017177066687911, -0.10141614984159, 0.99484407574561, 0.097553254298482, -0.10170938971513, 0.99001947587891, 0.19196303522488, -0.10003394863748, 0.97629063409788, 0.27974840137536, -0.097335167207941, 0.95512653462907, 0.35990826820895, -0.093946123749935, 0.92824574564454, 0.43339235817141, -0.090807141146948, 0.89661871885175, 0.5009392234462, -0.087769872261155, 0.86102052468938, 0.56324220105994, -0.085193316555122, 0.82188826598236, 0.62015393716623, -0.08185132576752, 0.78019834317135, -0.60453498384506, 0.0017534749197169, 0.79657666211932, -0.54795308050288, 0.0019190242367802, 0.83650686722428, -0.48678558751903, 0.0018125044973696, 0.87351960860143, -0.42074736281425, 0.0017406744057192, 0.90717618285395, -0.34956809139784, 0.0015982107976109, 0.9369095981997, -0.27250572340337, 0.0013800379423635, 0.96215317190543, -0.18871081631397, 0.00097430513853702, 0.98203221868512, -0.097563453907807, 0.00058126814689062, 0.99522913672627, -0.00047507344395353, -0.00028078788783147, 0.99999984773168, 0.098904010238072, -0.00058180614227329, 0.99509680848671, 0.19419772611147, -0.00079963719329112, 0.98096208069094, 0.28238055156247, -0.00090934663035081, 0.95930203647651, 0.36267404652775, -0.0011705852606918, 0.93191532110237, 0.43594915351216, -0.0013579804735627, 0.89997027253186, 0.50350053126417, -0.0017124651531993, 0.86399321900105, 0.56548557999579, -0.0019073905826808, 0.82475597644272, 0.62302499533586, -0.0026268925688553, 0.78219751637434, -0.60336920168812, 0.079988302569029, 0.79344027998732, -0.54678700420837, 0.08273175965705, 0.83317430827702, -0.48543462292347, 0.085633785223369, 0.87006900973168, -0.41938363966673, 0.088674669776908, 0.90346785538825, -0.34795042325352, 0.09190512143889, 0.93299729453573, -0.27096966095331, 0.095153792767757, 0.95787326853022, -0.18715948488214, 0.098341093330156, 0.97739467799926, -0.096745941786536, 0.10074474715615, 0.99019731299791, 1.3730777167575e-05, 0.10224348974085, 0.99475940237682, 0.098348548776397, 0.10154377046823, 0.98995778982372, 0.19307010544201, 0.099029151153722, 0.97617475976712, 0.28097111766388, 0.09564911201401, 0.95493794479518, 0.36128349144785, 0.091966730982426, 0.92790967189681, 0.43476442503797, 0.088259179260334, 0.89620879933065, 0.50234747615878, 0.084703291042368, 0.86050703988027, 0.56461092323569, 0.08122331871997, 0.821350885955, 0.62142338354404, 0.078269817487389, 0.77955552339469, -0.59977522730989, 0.15699679631825, 0.78461562733046, -0.54282449059575, 0.16237685638376, 0.8239995927914, -0.48141717765264, 0.16807428851259, 0.8602258625512, -0.41502628873979, 0.17397318104297, 0.89302100307477, -0.34355013973679, 0.17991277708398, 0.92174003608868, -0.26634982179325, 0.18601989348123, 0.94575597891842, -0.18307125227139, 0.19167457155557, 0.96423325767719, -0.093625431184466, 0.19640532941161, 0.97604263493673, 0.00056581531921107, 0.19879969158023, 0.98003998004195, 0.09619397699434, 0.19794781363525, 0.9754811027729, 0.18903241695214, 0.19432813540762, 0.96255042524027, 0.27610939437897, 0.18883954238631, 0.94239229069799, 0.3562588325328, 0.18243491810909, 0.91640446577747, 0.42996276002139, 0.17590072937967, 0.8855455710456, 0.49784483817453, 0.169495618091, 0.85054203455851, 0.56020068110842, 0.16321346598259, 0.81211856363933, 0.61796555017006, 0.15745300348607, 0.77027730753037, -0.59344423007425, 0.2321646944299, 0.77066432410736, -0.53653651553437, 0.23980580915012, 0.80908698011777, -0.47488625835712, 0.2478385400577, 0.84442826793378, -0.40842799288346, 0.25592568219498, 0.87617841780212, -0.33679705222366, 0.2640611028748, 0.9037917235525, -0.25996437027823, 0.2718722297348, 0.92655491843974, -0.17748780365353, 0.27903190793309, 0.94373686688052, -0.089937805789456, 0.28443713143317, 0.95446671463799, 0.0012016455036634, 0.28722131970837, 0.95786349212874, 0.093451729801734, 0.28659832055311, 0.95348213242472, 0.18359257381195, 0.28256919235493, 0.94151389706843, 0.26903860172353, 0.27605639651161, 0.9227194030298, 0.34865549382496, 0.26821106930625, 0.89805465809564, 0.42235571868831, 0.25980141159694, 0.86840018046032, 0.49040107034933, 0.25117584502096, 0.83451631804192, 0.5530642315081, 0.24266772433992, 0.79701463687315, 0.61061139190609, 0.23414474306187, 0.75652492845312, -0.58446503463329, 0.30341260040247, 0.7525565873794, -0.52825973802562, 0.31413401145219, 0.78883551646085, -0.46650459594303, 0.32394657766831, 0.8230626202064, -0.40008266781332, 0.33377417438346, 0.8535389032903, -0.32882127669184, 0.34318917638179, 0.87982825438215, -0.25248233133262, 0.35204946065068, 0.90128455529894, -0.17147595701658, 0.3596328373703, 0.91720238685376, -0.08631644218403, 0.36535206348441, 0.92685885738681, 0.001744755037815, 0.36803879788706, 0.92980879705438, 0.090466883787115, 0.36744569106976, 0.92563459693991, 0.17748656349881, 0.36346090626751, 0.91454616580716, 0.26092822803705, 0.35684054505228, 0.89698455127178, 0.33949301478333, 0.34838147442412, 0.87371324883591, 0.41277571037663, 0.33887640603206, 0.84544603278853, 0.48072317624745, 0.32886329876545, 0.81286786044453, 0.54340981606919, 0.31850883291137, 0.77669678456723, 0.60144782450456, 0.30875207979332, 0.73683964851386, -0.57753085675917, 0.3749153889359, 0.72518725900974, -0.51805005275704, 0.38437688489618, 0.764119462647, -0.45682324332602, 0.39593154871906, 0.79658692751389, -0.39060225765983, 0.40673529406726, 0.82583065871343, -0.31996045324995, 0.41713062673656, 0.85066288774958, -0.24484522745928, 0.42637686701636, 0.8707775731286, -0.16556198835984, 0.43411395542985, 0.88551358076056, -0.082801576639819, 0.43948871017998, 0.8944235979279, 0.0018404674900542, 0.44229847289094, 0.89686602876782, 0.08735705143286, 0.44166692470005, 0.89291605047225, 0.17135040885477, 0.43789647239407, 0.88254502256266, 0.25247065991783, 0.43137224107584, 0.86612733215726, 0.32954403085847, 0.42273520694237, 0.84421305162674, 0.40195148027714, 0.41275291420513, 0.81735551586699, 0.46939029483747, 0.40173740127982, 0.78630770791423, 0.5320918691632, 0.39039386406389, 0.75131276687653, 0.58922766450437, 0.37729273473133, 0.71446550070782, -0.55370564025101, 0.42273537751598, 0.71742934464004, -0.50766142997837, 0.45193263398963, 0.73350989553483, -0.44596295541891, 0.46247808857699, 0.76631002863084, -0.38080736199698, 0.47538124535566, 0.79309420916623, -0.31102282582976, 0.48532974676749, 0.81714125995171, -0.23699991303173, 0.49508547337503, 0.83589557677737, -0.15975591468744, 0.50269749155804, 0.84957240992374, -0.080042512726129, 0.50856178936964, 0.85729697453662, 0.0029940997947606, 0.51040808841544, 0.85992710077455, 0.083892933248918, 0.5100749526961, 0.85602892380045, 0.16529082544537, 0.50641797077325, 0.84629769106473, 0.24387489056127, 0.50018420636091, 0.83086749693368, 0.31936279000756, 0.49188705036556, 0.80997193657636, 0.39024252092137, 0.48114875417805, 0.7849883127906, 0.45749239294863, 0.47020179339268, 0.75472576733834, 0.51785187826028, 0.4569953043983, 0.72317682757414, 0.58064806199117, 0.44791564288711, 0.67986719656337])yaml";
  
  // Save the YAML text to a temporary file
  const char* file_path = "/tmp/__yaml_camera_read_test.yaml";
  FILE* file = fopen(file_path, "wb");
  if (!file) {
    std::cout << "Writing temporary file failed!" << std::endl;
    return false;
  }
  fwrite(yaml_text, 1, strlen(yaml_text), file);
  fclose(file);
  
  // Load the camera from the file and remove the temporary file again
  CentralGenericCamera<double> camera;
  std::string error_reason;
  bool result = camera.Read(file_path, &error_reason);
  remove(file_path);
  if (!result) {
    std::cout << "Reading the camera failed! Reason: " << error_reason << std::endl;
    return false;
  }
  
  // Check the values
  if (camera.width() != 640 ||
      camera.height() != 480 ||
      camera.calibration_min_x() != 15 ||
      camera.calibration_min_y() != 16 ||
      camera.calibration_max_x() != 624 ||
      camera.calibration_max_y() != 464 ||
      camera.grid_width() != 17 ||
      camera.grid_height() != 13) {
    std::cout << "Camera attribute not read correctly!" << std::endl;
    return false;
  }
  
  if (std::fabs(camera.grid_value(camera.grid_width() - 1, camera.grid_height() - 1).z() - 0.67986719656337) > 1e-3) {
    std::cout << "Grid value not read correctly!" << std::endl;
    return false;
  }
  
  return true;
}


bool TestNoncentralGenericCameraReprojection() {
  // Initialize a camera arbitrarily for testing
  NoncentralGenericCamera<double> camera(
      /*width*/ 640,
      /*height*/ 480,
      /*calibration_min_x*/ 0,
      /*calibration_min_y*/ 0,
      /*calibration_max_x*/ 639,
      /*calibration_max_y*/ 479,
      /*grid_width*/ 8,
      /*grid_height*/ 8);
  for (int y = 0; y < camera.grid_height(); ++ y) {
    for (int x = 0; x < camera.grid_width(); ++ x) {
      camera.direction_grid_value(x, y) = Eigen::Vector3d(x, y, 1).normalized();
      camera.point_grid_value(x, y) = Eigen::Vector3d(0.2 * x, 0.1 * y, 0.01 * x);
    }
  }
  
  // Verify that un-projecting a pixel and projecting the result again returns
  // the original pixel coordinate.
  int step = 10;
  for (int y = 0; y < camera.height(); y += step) {
    for (int x = 0; x < camera.width(); x += step) {
      // Note that 0.5 has to be added to x and y to get the center of the pixel.
      Eigen::Vector2d pixel(x + 0.5, y + 0.5);
      
      Eigen::ParametrizedLine<double, 3> line;
      if (!camera.Unproject(pixel, &line)) {
        std::cout << "Unprojection failed! Pixel: (" << pixel.transpose() << ")" << std::endl;
        return false;
      }
      
      Eigen::Vector2d reprojected_pixel;
      if (!camera.Project(line.origin() + x * line.direction(), &reprojected_pixel)) {  // NOTE: Arbitrarily adding x times the line direction for testing
        std::cout << "Reprojection failed! Pixel: (" << pixel.transpose() << "), Origin: (" << line.origin().transpose() << "), Direction: (" << line.direction().transpose() << ")" << std::endl;
        return false;
      }
      
      if ((pixel - reprojected_pixel).norm() > 1e-3) {
        std::cout << "Reprojection gave an incorrect result! Pixel: (" << pixel.transpose() << "), Reprojected pixel: (" << reprojected_pixel.transpose() << ")" << std::endl;
        return false;
      }
    }
  }
  
  return true;
}


bool TestNoncentralGenericCameraRead() {
  const char* yaml_text = R"yaml(type : NoncentralGenericModel
width : 640
height : 480
calibration_min_x : 15
calibration_min_y : 16
calibration_max_x : 624
calibration_max_y : 464
grid_width : 4
grid_height : 4
# The grid is stored in row-major order, top to bottom. Each row is stored left to right. Each grid point is stored as x, y, z.
point_grid : [0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 10, 11, 12]
direction_grid : [0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 20, 21, 20])yaml";
  
  // Save the YAML text to a temporary file
  const char* file_path = "/tmp/__yaml_camera_read_test.yaml";
  FILE* file = fopen(file_path, "wb");
  if (!file) {
    std::cout << "Writing temporary file failed!" << std::endl;
    return false;
  }
  fwrite(yaml_text, 1, strlen(yaml_text), file);
  fclose(file);
  
  // Load the camera from the file and remove the temporary file again
  NoncentralGenericCamera<double> camera;
  std::string error_reason;
  bool result = camera.Read(file_path, &error_reason);
  remove(file_path);
  if (!result) {
    std::cout << "Reading the camera failed! Reason: " << error_reason << std::endl;
    return false;
  }
  
  // Check the values
  if (camera.width() != 640 ||
      camera.height() != 480 ||
      camera.calibration_min_x() != 15 ||
      camera.calibration_min_y() != 16 ||
      camera.calibration_max_x() != 624 ||
      camera.calibration_max_y() != 464 ||
      camera.grid_width() != 4 ||
      camera.grid_height() != 4) {
    std::cout << "Camera attribute not read correctly!" << std::endl;
    return false;
  }
  
  if ((camera.point_grid_value(camera.grid_width() - 1, camera.grid_height() - 1) - Eigen::Vector3d(10, 11, 12)).norm() > 1e-5) {
    std::cout << "Point grid value not read correctly!" << std::endl;
    return false;
  }
  if ((camera.direction_grid_value(camera.grid_width() - 1, camera.grid_height() - 1) - Eigen::Vector3d(20, 21, 20).normalized()).norm() > 1e-5) {
    std::cout << "Direction grid value not read correctly!" << std::endl;
    return false;
  }
  
  return true;
}

void bilinear_interpolate(const cv::Mat &src_img, const double src_x, const double src_y, uchar &color) {
  double src_x_f = floor(src_x);
  double src_x_c = ceil(src_x);
  double src_y_f = floor(src_y);
  double src_y_c = ceil(src_y);
  double w1 = (src_x_c - src_x) / (src_x_c - src_x_f);
  double w2 = (src_x - src_x_f) / (src_x_c - src_x_f);
  double w3 = (src_y_c - src_y) / (src_y_c - src_y_f);
  double w4 = (src_y - src_y_f) / (src_y_c - src_y_f);
  uchar q12 = src_img.at<uchar>(src_y_f, src_x_f);
  uchar q22 = src_img.at<uchar>(src_y_f, src_x_c);
  uchar q11 = src_img.at<uchar>(src_y_c, src_x_f);
  uchar q21 = src_img.at<uchar>(src_y_c, src_x_c);
  color = w3 * (w1 * q12 + w2 * q22) + w4 * (w1 * q11 + w2 * q21);
}

// cv::Matx33d quaternion2rmat(const double x, const double y, const double z, const double w) {
//   cv::Matx33d rmat;
//   rmat(0, 0) = 1 - 2*y*y - 2*z*z;
//   rmat(0, 1) = 2*x*y - 2*z*w;
//   rmat(0, 2) = 2*x*z + 2*y*w;
//   rmat(1, 0) = 2*x*y + 2*z*w;
//   rmat(1, 1) = 1 - 2*x*x - 2*z*z;
//   rmat(1, 2) = 2*y*z - 2*x*w;
//   rmat(2, 0) = 2*x*z - 2*y*w;
//   rmat(2, 1) = 2*y*z + 2*x*w;
//   rmat(2, 2) = 1 - 2*x*x - 2*y*y;
//   return rmat;
// }

cv::Matx33d quaternion2rmat(const Eigen::Quaterniond &q) {
  cv::Matx33d rmat;
  auto rmat_eigen = q.normalized().toRotationMatrix();
  // auto rmat_eigen = q.toRotationMatrix();
  rmat(0, 0) = rmat_eigen(0, 0);
  rmat(0, 1) = rmat_eigen(0, 1);
  rmat(0, 2) = rmat_eigen(0, 2);
  rmat(1, 0) = rmat_eigen(1, 0);
  rmat(1, 1) = rmat_eigen(1, 1);
  rmat(1, 2) = rmat_eigen(1, 2);
  rmat(2, 0) = rmat_eigen(2, 0);
  rmat(2, 1) = rmat_eigen(2, 1);
  rmat(2, 2) = rmat_eigen(2, 2);
  return rmat;
}

// TODO
void myUndistortPoints(InputArray distorted, OutputArray undistorted, InputArray K, InputArray R, InputArray P = noArray()) {
    const cv::Vec2d* srcd = distorted.getMat().ptr<cv::Vec2d>();
    cv::Vec2d* dstd = undistorted.getMat().ptr<cv::Vec2d>();
    dstd[0] = Vec2d(srcd[0][0], srcd[0][1]);
    dstd[1] = Vec2d(srcd[1][0] * 1.5, srcd[1][1]);
    dstd[2] = Vec2d(srcd[2][0], srcd[2][1]);
    dstd[3] = Vec2d(srcd[3][0] * (-0.5), srcd[3][1]);
}

void myEstimateNewCameraMatrixForUndistortRectify(InputArray K, const Size &image_size, InputArray R,
    OutputArray P, double balance, const Size& new_size, double fov_scale, InputArray ref_points) {
    CV_Assert(K.size() == Size(3, 3) && (K.depth() == CV_32F || K.depth() == CV_64F));

    int w = image_size.width, h = image_size.height;
    balance = std::min(std::max(balance, 0.0), 1.0);

    cv::Mat points(1, 4, CV_64FC2);
    Vec2d* pptr = points.ptr<Vec2d>();
    if (ref_points.empty()) {
        pptr[0] = Vec2d(w/2, 0);
        pptr[1] = Vec2d(w, h/2);
        pptr[2] = Vec2d(w/2, h);
        pptr[3] = Vec2d(0, h/2);
    } else {
        Vec2d* rpptr = ref_points.getMat().ptr<Vec2d>();
        pptr[0] = rpptr[0];
        pptr[1] = rpptr[1];
        pptr[2] = rpptr[2];
        pptr[3] = rpptr[3];
    }

    myUndistortPoints(points, points, K, R);
    cv::Scalar center_mass = mean(points);
    cv::Vec2d cn(center_mass.val);

    double aspect_ratio = (K.depth() == CV_32F) ? K.getMat().at<float >(0,0)/K.getMat().at<float> (1,1)
                                                : K.getMat().at<double>(0,0)/K.getMat().at<double>(1,1);

    // convert to identity ratio
    cn[0] *= aspect_ratio;
    for(size_t i = 0; i < points.total(); ++i)
        pptr[i][1] *= aspect_ratio;

    double minx = DBL_MAX, miny = DBL_MAX, maxx = -DBL_MAX, maxy = -DBL_MAX;
    for(size_t i = 0; i < points.total(); ++i)
    {
        miny = std::min(miny, pptr[i][1]);
        maxy = std::max(maxy, pptr[i][1]);
        minx = std::min(minx, pptr[i][0]);
        maxx = std::max(maxx, pptr[i][0]);
    }

    double f1 = w * 0.5/(cn[0] - minx);
    double f2 = w * 0.5/(maxx - cn[0]);
    double f3 = h * 0.5 * aspect_ratio/(cn[1] - miny);
    double f4 = h * 0.5 * aspect_ratio/(maxy - cn[1]);

    printf("f1: %.2lf\n", f1);
    printf("f2: %.2lf\n", f2);

    double fmin = std::min(f1, std::min(f2, std::min(f3, f4)));
    double fmax = std::max(f1, std::max(f2, std::max(f3, f4)));

    double f = balance * fmin + (1.0 - balance) * fmax;
    f *= fov_scale > 0 ? 1.0/fov_scale : 1.0;

    cv::Vec2d new_f(f, f), new_c = -cn * f + Vec2d(w, h * aspect_ratio) * 0.5;

    // restore aspect ratio
    new_f[1] /= aspect_ratio;
    new_c[1] /= aspect_ratio;

    if (!new_size.empty())
    {
        double rx = new_size.width /(double)image_size.width;
        double ry = new_size.height/(double)image_size.height;

        new_f[0] *= rx;  new_f[1] *= ry;
        new_c[0] *= rx;  new_c[1] *= ry;
    }

    Mat(Matx33d(new_f[0], 0, new_c[0],
                0, new_f[1], new_c[1],
                0,        0,       1)).convertTo(P, P.empty() ? K.type() : P.type());
}

void myStereoRectify(InputArray K1, InputArray K2, const Size& imageSize,
        InputArray _R, InputArray _tvec, OutputArray R1, OutputArray R2, OutputArray P1, OutputArray P2,
        OutputArray Q, int flags, const Size& newImageSize, double balance = 0.0, double fov_scale = 1.0, InputArray ref_points = noArray()) {

    CV_Assert((_R.size() == Size(3, 3) || _R.total() * _R.channels() == 3) && (_R.depth() == CV_32F || _R.depth() == CV_64F));
    CV_Assert(_tvec.total() * _tvec.channels() == 3 && (_tvec.depth() == CV_32F || _tvec.depth() == CV_64F));

    // rotation matrix to rotation vector
    Matx33d rmat;
    _R.getMat().convertTo(rmat, CV_64F);
    Vec3d rvec;
    rvec = Affine3d(rmat).rvec();

    Vec3d tvec;
    _tvec.getMat().convertTo(tvec, CV_64F);

    // rectification algorithm
    rvec *= -0.5;  // get average rotation

    // rotation vector back to rotation matrix
    Matx33d r_r;
    Rodrigues(rvec, r_r);

    Vec3d t = r_r * tvec;
    Vec3d uu(t[0] > 0 ? 1 : -1, 0, 0);

    // calculate global Z rotation
    Vec3d ww = t.cross(uu);
    double nw = norm(ww);
    if (nw > 0.0)
        ww *= acos(fabs(t[0])/cv::norm(t))/nw;

    Matx33d wr;
    Rodrigues(ww, wr);

    // apply to both views
    Matx33d ri1 = wr * r_r.t();
    Mat(ri1, false).convertTo(R1, R1.empty() ? CV_64F : R1.type());
    Matx33d ri2 = wr * r_r;
    Mat(ri2, false).convertTo(R2, R2.empty() ? CV_64F : R2.type());
    Vec3d tnew = ri2 * tvec;

    // calculate projection/camera matrices. these contain the relevant rectified image internal params (fx, fy=fx, cx, cy)
    Matx33d newK1, newK2;
    myEstimateNewCameraMatrixForUndistortRectify(K1, imageSize, R1, newK1, balance, newImageSize, fov_scale, ref_points);
    myEstimateNewCameraMatrixForUndistortRectify(K2, imageSize, R2, newK2, balance, newImageSize, fov_scale, ref_points);

    double fc_new = std::min(newK1(1,1), newK2(1,1));
    Point2d cc_new[2] = { Vec2d(newK1(0, 2), newK1(1, 2)), Vec2d(newK2(0, 2), newK2(1, 2)) };

    // Vertical focal length must be the same for both images to keep the epipolar constraint use fy for fx also.
    // For simplicity, set the principal points for both cameras to be the average
    // of the two principal points (either one of or both x- and y- coordinates)
    if( flags & cv::CALIB_ZERO_DISPARITY )
        cc_new[0] = cc_new[1] = (cc_new[0] + cc_new[1]) * 0.5;
    else
        cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;

    Mat(Matx34d(fc_new, 0, cc_new[0].x, 0,
                0, fc_new, cc_new[0].y, 0,
                0,      0,           1, 0), false).convertTo(P1, P1.empty() ? CV_64F : P1.type());

    Mat(Matx34d(fc_new, 0, cc_new[1].x, tnew[0]*fc_new, // baseline * focal length;,
                0, fc_new, cc_new[1].y,              0,
                0,      0,           1,              0), false).convertTo(P2, P2.empty() ? CV_64F : P2.type());

    if (Q.needed())
        Mat(Matx44d(1, 0, 0,           -cc_new[0].x,
                    0, 1, 0,           -cc_new[0].y,
                    0, 0, 0,            fc_new,
                    0, 0, -1./tnew[0], (cc_new[0].x - cc_new[1].x)/tnew[0]), false).convertTo(Q, Q.empty() ? CV_64F : Q.depth());
}

void myInitUndistortRectifyMap(
    const CentralGenericCamera<double>& camera,
    InputArray K, InputArray R, InputArray P,
    const cv::Size& size, int m1type, OutputArray map1, OutputArray map2)
{
    CV_Assert( m1type == CV_16SC2 || m1type == CV_32F || m1type <=0 );
    map1.create( size, m1type <= 0 ? CV_16SC2 : m1type );
    map2.create( size, map1.type() == CV_16SC2 ? CV_16UC1 : CV_32F );

    CV_Assert(K.depth() == CV_32F || K.depth() == CV_64F);
    CV_Assert((P.empty() || P.depth() == CV_32F || P.depth() == CV_64F) && (R.empty() || R.depth() == CV_32F || R.depth() == CV_64F));
    CV_Assert(K.size() == Size(3, 3));
    CV_Assert(R.empty() || R.size() == Size(3, 3) || R.total() * R.channels() == 3);
    CV_Assert(P.empty() || P.size() == Size(3, 3) || P.size() == Size(4, 3));

    cv::Vec2d f, c;
    if (K.depth() == CV_32F)
    {
        Matx33f camMat = K.getMat();
        f = Vec2f(camMat(0, 0), camMat(1, 1));
        c = Vec2f(camMat(0, 2), camMat(1, 2));
    }
    else
    {
        Matx33d camMat = K.getMat();
        f = Vec2d(camMat(0, 0), camMat(1, 1));
        c = Vec2d(camMat(0, 2), camMat(1, 2));
    }

    cv::Matx33d RR  = cv::Matx33d::eye();
    if (!R.empty() && R.total() * R.channels() == 3)
    {
        cv::Vec3d rvec;
        R.getMat().convertTo(rvec, CV_64F);
        RR = Affine3d(rvec).rotation();
    }
    else if (!R.empty() && R.size() == Size(3, 3))
        R.getMat().convertTo(RR, CV_64F);

    cv::Matx33d PP = cv::Matx33d::eye();
    if (!P.empty())
        P.getMat().colRange(0, 3).convertTo(PP, CV_64F);

    cv::Matx33d iR = (PP * RR).inv(cv::DECOMP_SVD);

    for( int i = 0; i < size.height; ++i)
    {
        float* m1f = map1.getMat().ptr<float>(i);
        float* m2f = map2.getMat().ptr<float>(i);
        short*  m1 = (short*)m1f;
        ushort* m2 = (ushort*)m2f;

        double _x = i*iR(0, 1) + iR(0, 2),
               _y = i*iR(1, 1) + iR(1, 2),
               _w = i*iR(2, 1) + iR(2, 2);

        for( int j = 0; j < size.width; ++j)
        {
            double u, v;
            printf("(%d, %d) -> (%.2lf, %.2lf, %.2lf)\n", i, j, _x, _y, _w);
            if( _w <= 0)
            {
                u = (_x > 0) ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
                v = (_y > 0) ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
            }
            else
            {
                double x = _x/_w, y = _y/_w;

                // double r = sqrt(x*x + y*y);
                // double theta = atan(r);

                // double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta4*theta4;
                // double theta_d = theta * (1 + k[0]*theta2 + k[1]*theta4 + k[2]*theta6 + k[3]*theta8);

                // double scale = (r == 0) ? 1.0 : theta_d / r;
                // u = f[0]*x*scale + c[0];
                // v = f[1]*y*scale + c[1];

                CentralGenericCamera<double>::PointT d((x - c[0]) / f[0], (y - c[1]) / f[1], 1);
                CentralGenericCamera<double>::PixelT p2;
                camera.Project(d, &p2);
                u = p2(0, 0);
                v = p2(0, 1);
                // printf("(%d, %d) -> (%.2lf, %.2lf)\n", i, j, u, v);
            }

            if( m1type == CV_16SC2 )
            {
                int iu = cv::saturate_cast<int>(u*cv::INTER_TAB_SIZE);
                int iv = cv::saturate_cast<int>(v*cv::INTER_TAB_SIZE);
                m1[j*2+0] = (short)(iu >> cv::INTER_BITS);
                m1[j*2+1] = (short)(iv >> cv::INTER_BITS);
                m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE-1))*cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE-1)));
            }
            else if( m1type == CV_32FC1 )
            {
                m1f[j] = (float)u;
                m2f[j] = (float)v;
            }

            _x += iR(0, 0);
            _y += iR(1, 0);
            _w += iR(2, 0);
        }
    }
}

bool get_extrinsics(const Eigen::Quaterniond &q0, const Eigen::Quaterniond &q1,
        const cv::Vec3d &t0, const cv::Vec3d &t1,
        cv::Mat &R, cv::Mat &T) {
    // quaternion to ratation matrix
    cv::Matx33d r0 = quaternion2rmat(q0);
    cv::Matx33d r1 = quaternion2rmat(q1);
    R = Mat(r1 * r0.inv());

    // translation vector
    cv::Vec3d t = -r1 * r0.inv() * t0 + t1;
    T.create(3, 1, CV_64FC1);
    T.at<double>(0, 0) = t[0];
    T.at<double>(1, 0) = t[1];
    T.at<double>(2, 0) = t[2];

    return true;
}

bool get_intrinsics(const double f, const Size &size, Mat &K0, Mat &K1) {
    K0.create(3, 3, CV_64FC1);
    K0.at<double>(0, 0) = f;
    K0.at<double>(1, 1) = f;
    K0.at<double>(0, 2) = size.width / 2;
    K0.at<double>(1, 2) = size.height / 2;
    K0.at<double>(2, 2) = 1;

    K1.create(3, 3, CV_64FC1);
    K1.at<double>(0, 0) = f;
    K1.at<double>(1, 1) = f;
    K1.at<double>(0, 2) = size.width / 2;
    K1.at<double>(1, 2) = size.height / 2;
    K1.at<double>(2, 2) = 1;

    return true;
}

bool test_undistortion() {
  // Load the central_generic model.
  const char* file_path = "/Users/rankencheng/Downloads/test/result_15px_central_generic_20/intrinsics1.yaml";
  CentralGenericCamera<double> camera;
  std::string error_reason;
  bool result = camera.Read(file_path, &error_reason);
  if (!result) {
    std::cout << "Reading the camera failed! Reason: " << error_reason << std::endl;
    return false;
  }

  // Load fx, fy, cx, cy from the central_opencv model.
  double parameters[12] = { 277.43476524575, 277.29416182849, 307.81265362247, 233.0392096016, 0.93640031669187, 0.35692232820102, 0.015722514959806, 1.2608235807082, 0.59007496453161, 0.079399687311869, -0.00027415199228906, 5.6684176272052e-05 };
  const double& fx = parameters[0];
  const double& fy = parameters[1];
  const double& cx = parameters[2];
  const double& cy = parameters[3];

  cv::Mat src_img = cv::imread("/Users/rankencheng/Downloads/test/right/1.jpg", cv::IMREAD_GRAYSCALE);
  cv::Mat dst_img(src_img.rows * 2, src_img.cols * 2, CV_8U);  // double the width and height to obtain a larger FOV
  const int x_begin = 0 - camera.width() * 0.5;
  const int x_end = camera.width() * 1.5;
  const int y_begin = 0 - camera.height() * 0.5;
  const int y_end = camera.height() * 1.5;

  // For each pixel p in this pinhole calibration.
  for (int x = x_begin; x < x_end; ++x) {
    for (int y = y_begin; y < y_end; ++y) {

      // Un-project the pixel coordinate of p to a direction d.
      // The unprojection is simply done by applying the inverse pinhole matrix to pixel (p.x, p.y)^T
      // to get the direction ((p.x - c_x) / f_x, (p.y - c_y) / f_y, 1)^T.
      CentralGenericCamera<double>::PointT d((x - cx) / fx, (y - cy) / fy, 1);

      // Project this direction d to a pixel coordinate p2 using the calibrated camera model.
      CentralGenericCamera<double>::PixelT p2;
      camera.Project(d, &p2);

      // Interpolate the pixel color at the obtained pixel coordinate p2 in the original image,
      // and set the current pixel p in the undistorted image to this color.
      // Bilinear interpolation is used.
      uchar color;
      double src_x = p2(0, 0);
      double src_y = p2(1, 0);
      if (src_x < 0 || src_x >= src_img.cols) continue;
      if (src_y < 0 || src_y >= src_img.rows) continue;
      bilinear_interpolate(src_img, src_x, src_y, color);
      dst_img.at<uchar>(y + camera.height() * 0.5, x + camera.width() * 0.5) = color;

    }
  }

  cv::imwrite("/Users/rankencheng/Downloads/test/right/1_rectified.jpg", dst_img);
  return true;
}

int main(int /*argc*/, char** /*argv*/) {
  bool result;
  
  // Test CentralGenericCamera
  result = TestCentralGenericCameraReprojection();
  std::cout << "TestCentralGenericCameraReprojection: " << (result ? "success" : "failure") << std::endl;
  
  result = TestCentralGenericCameraRead();
  std::cout << "TestCentralGenericCameraRead: " << (result ? "success" : "failure") << std::endl;
  
  // Test NoncentralGenericCamera
  result = TestNoncentralGenericCameraReprojection();
  std::cout << "TestNoncentralGenericCameraReprojection: " << (result ? "success" : "failure") << std::endl;
  
  result = TestNoncentralGenericCameraRead();
  std::cout << "TestNoncentralGenericCameraRead: " << (result ? "success" : "failure") << std::endl;
  
  // test_undistortion();

  // load camera 0
  double tx0 = -0.0058676120060024;
  double ty0 = 0.0068879842290985;
  double tz0 = 0.020930250549758;
  double qx0 = 0.014966896212563;
  double qy0 = 0.019633204980285;
  double qz0 = 0.0008399391155767;
  double qw0 = 0.99969486533754;
  // load camera 1
  double tx1 = -0.12498542572133;
  double ty1 = 0.0053752520414682;
  double tz1 = 0.031373976496008;
  double qx1 = 0.0046813344329794;
  double qy1 = 0.034633731298288;
  double qz1 = 0.0029112873180275;
  double qw1 = 0.99938486789146;

  Eigen::Quaterniond q0(qx0, qy0, qz0, qw0), q1(qx1, qy1, qz1, qw1);
  cv::Vec3d t0(tx0, ty0, tz0), t1(tx1, ty1, tz1);
  Mat R, T;
  get_extrinsics(q0, q1, t0, t1, R, T);
  std::cout << "R: " << R << std::endl;
  std::cout << "T: " << T << std::endl;

  Mat K0, K1;
  Size image_size(640, 480), new_image_size(640 * 1.5, 480);
  const double f = 2.0 / 2.9e-3;
  get_intrinsics(f, image_size, K0, K1);
  std::cout << "K0: " << K0 << std::endl;
  std::cout << "K1: " << K1 << std::endl;

  Mat R0, R1, P0, P1, Q;
  myStereoRectify(K0, K1, image_size, R, T, R0, R1, P0, P1, Q, CALIB_ZERO_DISPARITY, new_image_size);
  std::cout << "R0: " << R0 << std::endl;
  std::cout << "R1: " << R1 << std::endl;
  std::cout << "P0: " << P0 << std::endl;
  std::cout << "P1: " << P1 << std::endl;
  std::cout << "Q: " << Q << std::endl;
  return 0;

  CentralGenericCamera<double> camera0, camera1;
  std::string error_reason[2];
  camera0.Read("/Users/rankencheng/Downloads/test/result_15px_central_generic_20/intrinsics0.yaml", &error_reason[0]);
  camera1.Read("/Users/rankencheng/Downloads/test/result_15px_central_generic_20/intrinsics1.yaml", &error_reason[1]);
  Mat rmap[2][2];
  myInitUndistortRectifyMap(camera0, K0, R0, P0, new_image_size, CV_16SC2, rmap[0][0], rmap[0][1]);
  myInitUndistortRectifyMap(camera1, K1, R1, P1, new_image_size, CV_16SC2, rmap[1][0], rmap[1][1]);

  Mat measure_image[2], rectified_images[2];
  measure_image[0] = cv::imread("/Users/rankencheng/Downloads/test/left/1.jpg");
  measure_image[1] = cv::imread("/Users/rankencheng/Downloads/test/right/1.jpg");
  remap(measure_image[0], rectified_images[0], rmap[0][0], rmap[0][1], INTER_LINEAR);
  remap(measure_image[1], rectified_images[1], rmap[1][0], rmap[1][1], INTER_LINEAR);
  imwrite("/Users/rankencheng/Downloads/test/left/1_rectified_test.jpg", rectified_images[0]);
  imwrite("/Users/rankencheng/Downloads/test/right/1_rectified_test.jpg", rectified_images[1]);

  return 0;
}
