# planning模块详解
## 前言

apollo5.0版本的planning模块的详解。本文主要内容如下：
* planning模块的加载
* planning模块的初始化
* planning算法的调用步骤
* scenarios内部详解等内容

## 1. planning模块加载
51的apollo5.0启动脚本是在`/apollo/run.sh`中，代码如下：
```bash
export LD_LIBRARY_PATH=/usr/local/lib64:$LD_LIBRARY_PATH
source /apollo/scripts/apollo_base.sh
cyber_launch start /apollo/modules/localization/launch/localization.launch &
sleep 1
cyber_launch start /apollo/modules/prediction/launch/prediction.launch &
sleep 1
cyber_launch start /apollo/modules/routing/launch/routing.launch &
sleep 1
cyber_launch start /apollo/modules/planning/launch/planning.launch &
sleep 1
cyber_launch start /apollo/modules/control/launch/control.launch &
sleep 1
cyber_launch start /apollo/modules/dreamview/launch/dreamview.launch &
```
其中planning模块的启动代码为：`cyber_launch start /apollo/modules/planning/launch/planning.launch &`
**这里`cybler_launch`是cyber自带的一个python写的启动工具。**
文件地址为`/apollo/cyber/tools/cyber_launch/cyber_launch`。
对比planning模块和dreamview模块的launch文件：
* planning模块：
```xml
<!-- /apollo/modules/planning/launch/planning.launch -->
<cyber>
    <module>
        <name>planning</name>
        <dag_conf>/apollo/modules/planning/dag/planning.dag</dag_conf>
        <process_name>planning</process_name>
    </module>
</cyber>
```
* dreamview模块：
```xml
<!-- /apollo/modules/dreamview/launch/dreamview.launch-->
<cyber>
    <module>
        <name>dreamview</name>
        <dag_conf></dag_conf>
        <type>binary</type>
        <process_name>
           /apollo/bazel-bin/modules/dreamview/dreamview --flagfile=/apollo/modules/common/data/global_flagfile.txt
        </process_name>
        <exception_handler>respawn</exception_handler>
    </module>
</cyber>
```
**注意到dreamview中有`<type>`标签。**
查看`cyber_launch`中的start函数:
```python
def start(launch_file=''):
    """
    Start all modules in xml config
    """
    pmon = ProcessMonitor()
    # Find launch file
    if launch_file[0] == '/':
        launch_file = launch_file
    elif launch_file == os.path.basename(launch_file):
        launch_file = os.path.join(cyber_path, 'launch', launch_file)
    else:
        if os.path.exists(os.path.join(g_pwd, launch_file)):
            launch_file = os.path.join(g_pwd, launch_file)
        else:
            logger.error('Cannot find launch file: %s ' % launch_file)
            sys.exit(1)
    logger.info('Launch file [%s]' % launch_file)
    logger.info('=' * 120)

    if not os.path.isfile(launch_file):
        logger.error('Launch xml file %s does not exist' % launch_file)
        sys.exit(1)

    try:
        tree = ET.parse(launch_file)
    except Exception:
        logger.error('Parse xml failed. illegal xml!')
        sys.exit(1)
    total_dag_num = 0
    dictionary = {}
    dag_dict = {}
    root1 = tree.getroot()
    for module in root1.findall('module'):
        dag_conf = module.find('dag_conf').text
        process_name = module.find('process_name').text
        process_type = module.find('type')
        if process_type is None:
            process_type = 'library'
        else:
            process_type = process_type.text
            if process_type is None:
                process_type = 'library'
            process_type = process_type.strip()
        if process_type != 'binary':
            if dag_conf is None or not dag_conf.strip():
                logger.error('Library dag conf is null')
                continue
            if process_name is None:
                process_name = 'mainboard_default_' + str(os.getpid())
            process_name = process_name.strip()
            if dictionary.has_key(str(process_name)):
                dictionary[str(process_name)] += 1
            else:
                dictionary[str(process_name)] = 1
            if not dag_dict.has_key(str(process_name)):
                dag_dict[str(process_name)] = [str(dag_conf)]
            else:
                dag_dict[str(process_name)].append(str(dag_conf))
            if dag_conf is not None:
                total_dag_num += 1

    process_list = []
    root = tree.getroot()
    for env in root.findall('environment'):
        for var in env.getchildren():
            os.environ[var.tag] = str(var.text)
    for module in root.findall('module'):
        module_name = module.find('name').text
        dag_conf = module.find('dag_conf').text
        process_name = module.find('process_name').text
        sched_name = module.find('sched_name')
        process_type = module.find('type')
        exception_handler = module.find('exception_handler')
        if process_type is None:
            process_type = 'library'
        else:
            process_type = process_type.text
            if process_type is None:
                process_type = 'library'
            process_type = process_type.strip()

        .......
        if process_name not in process_list:
            if process_type == 'binary':
                if len(process_name) == 0:
                    logger.error(
                        'Start binary failed. Binary process_name is null.')
                    continue
                pw = ProcessWrapper(
                    process_name.split()[0], 0, [
                        ""], process_name, process_type,
                    exception_handler)
            # Default is library
            else:
                pw = ProcessWrapper(
                    g_binary_name, 0, dag_dict[
                        str(process_name)], process_name,
                    process_type, sched_name, exception_handler)
            result = pw.start()
            if result != 0:
                logger.error(
                    'Start manager [%s] failed. Stop all!' % process_name)
                stop()
            pmon.register(pw)
            process_list.append(process_name)
            ......
```
planning模块的process_type是None，所以代码里将process_type赋值为library。后续调用ProcessWrapper的时候，利用process_type做判断，调用了
```python
pw = ProcessWrapper(
                    g_binary_name, 0, dag_dict[
                        str(process_name)], process_name,
                    process_type, sched_name, exception_handler)
```
这里的`g_binary_name`的定义为：
```python
g_binary_name = 'mainboard'
```
这里`ProcessWrapper`中有个start，里面建立了线程：
```python
th = threading.Thread(target=module_monitor, args=(self, ))
th.setDaemon(True)
th.start()
self.started = True
self.pid = self.popen.pid
logger.info('Start process [%s] successfully. pid: %d' %
(self.name, self.popen.pid))
logger.info('-' * 120)
return 0
```
实际调用了mainboard。mainboard.cc中主要进行cyber的初始化和模块的加载，相关代码如下：
```c++
  // in/apollo/cyber/mainboard/mainboard.cc
  ModuleController controller(module_args);
  if (!controller.Init()) {
    controller.Clear();
    AERROR << "module start error.";
    return -1;
  }
```
ModuleController类的实体controller的Init()函数就一句话：
```c++
//file in /apollo/cyber/mainboard/module_controller.h
inline bool ModuleController::Init() { return LoadAll(); }
```
查看LoadAll(),主要是对参数进行解析，重要的函数为：
```c++
if (!LoadModule(module_path)) {
      AERROR << "Failed to load module: " << module_path;
      return false;
    }
```
调用了LoadModule函数：
```c++
bool ModuleController::LoadModule(const DagConfig& dag_config) {
  const std::string work_root = common::WorkRoot();

  for (auto module_config : dag_config.module_config()) {
    ......

    class_loader_manager_.LoadLibrary(load_path);

    for (auto& component : module_config.components()) {
      const std::string& class_name = component.class_name();
      std::shared_ptr<ComponentBase> base =
          class_loader_manager_.CreateClassObj<ComponentBase>(class_name);
      if (base == nullptr || !base->Initialize(component.config())) {
        return false;
      }
      component_list_.emplace_back(std::move(base));
    }

    ......
  }
  return true;
}
```
重点为`LoadLibrary`和`std::shared_ptr<ComponentBase> base = class_loader_manager_.CreateClassObj<ComponentBase>(class_name);`
module_config是从dag文件中读取的，load_path是从module_config中读取的。查看CrateClassObj的函数的继承关系，最后找到这么一段：
* 代码1
```c++
//in/apollo/cyber/class_loader/class_loader.h
std::shared_ptr<Base> ClassLoader::CreateClassObj(
    const std::string& class_name) {
  if (!IsLibraryLoaded()) {
    LoadLibrary();
  }

  Base* class_object = utility::CreateClassObj<Base>(class_name, this);
  if (class_object == nullptr) {
    AWARN << "CreateClassObj failed, ensure class has been registered. "
          << "classname: " << class_name << ",lib: " << GetLibraryPath();
    return std::shared_ptr<Base>();
  }

  std::lock_guard<std::mutex> lck(classobj_ref_count_mutex_);
  classobj_ref_count_ = classobj_ref_count_ + 1;
  std::shared_ptr<Base> classObjSharePtr(
      class_object, std::bind(&ClassLoader::OnClassObjDeleter<Base>, this,
                              std::placeholders::_1));
  return classObjSharePtr;
}
```
再深入看：
* 代码2
```c++
//in/apollo/cyber/class_loader/utility/class_loader_utility.h
template <typename Base>
Base* CreateClassObj(const std::string& class_name, ClassLoader* loader) {
  GetClassFactoryMapMapMutex().lock();
  ClassClassFactoryMap& factoryMap =
      GetClassFactoryMapByBaseClass(typeid(Base).name());
  AbstractClassFactory<Base>* factory = nullptr;
  if (factoryMap.find(class_name) != factoryMap.end()) {
    factory = dynamic_cast<utility::AbstractClassFactory<Base>*>(
        factoryMap[class_name]);
  }
  GetClassFactoryMapMapMutex().unlock();

  Base* classobj = nullptr;
  if (factory && factory->IsOwnedBy(loader)) {
    classobj = factory->CreateObj();
  }

  return classobj;
}
```
代码2中的模板函数在代码1中被调用。通过factorymap建立以class_name命名的工厂实体，然后创建对象实体并返回一个Base类指针。整体代码层次为：
```C++
cyber_launch{
    start{
        ProcessWrapper{
            mainboard
        }
    }
}

in mainboard:

main{
    ModuleController.Init{
        LoadAll{
            LoadLibrary();
            CreateClassObj{
                {{
                    factory = dynamic_cast<utility::AbstractClassFactory<Base>*>(
                        factoryMap[class_name]);
                    classobj = factory->CreateObj();
                    return classobj;
                }}
            }
        }
    } 
    apollo::cyber::WaitForShutdown();
    return 0;
}
```
factoryMap的建立是在各个模块中单独编写的。下面分析planning模块是如何对factoryMap进行建立的。
* * *
planning整体封装在`planning_component`类中，头文件中有一句宏：
```c++
//file in /apollo/modules/planning/planning_component.h
CYBER_REGISTER_COMPONENT(PlanningComponent)
```
层层展开可以得到：
```c
#define CLASS_LOADER_REGISTER_CLASS_INTERNAL(Derived, Base, UniqueID)     \
  namespace {                                                             \
  struct ProxyType##UniqueID {                                            \
    ProxyType##UniqueID() {                                               \
      apollo::cyber::class_loader::utility::RegisterClass<Derived, Base>( \
          #Derived, #Base);                                               \
    }                                                                     \
  };                                                                      \
  static ProxyType##UniqueID g_register_class_##UniqueID;                 \
  }
```
其中调用了`RegisterClass`函数：
```C++
template <typename Derived, typename Base>
void RegisterClass(const std::string& class_name,
                   const std::string& base_class_name) {
  AINFO << "registerclass:" << class_name << "," << base_class_name << ","
        << GetCurLoadingLibraryName();

  utility::AbstractClassFactory<Base>* new_class_factrory_obj =
      new utility::ClassFactory<Derived, Base>(class_name, base_class_name);
  new_class_factrory_obj->AddOwnedClassLoader(GetCurActiveClassLoader());
  new_class_factrory_obj->SetRelativeLibraryPath(GetCurLoadingLibraryName());

  GetClassFactoryMapMapMutex().lock();
  ClassClassFactoryMap& factory_map =
      GetClassFactoryMapByBaseClass(typeid(Base).name());
  factory_map[class_name] = new_class_factrory_obj;
  GetClassFactoryMapMapMutex().unlock();
}
```
这里面就把planning和其设置好的new_class_factrory_obj存到一起去了。那边在调用f`actory_map[planning]`的时候就可以拿到`utility::AbstractClassFactory<Base>*`指针了。
## 2. planning模块的初始化
初始化函数在`/apollo/modules/planning/planning_component.cc`中，整体初始化过程如下：
```c++

bool PlanningComponent::Init {//file in/apollo/modules/planning/planning_component.cc
    if (FLAGS_use_navigation_mode) {
    planning_base_ = std::make_unique<NaviPlanning>();
    } else {
    planning_base_ = std::make_unique<OnLanePlanning>();
    }
    planning_base_->Init(config_){//这已经是OnLanePlanning的Init了,file in /apollo/modules/planning/on_lane_planning.cc
        PlanningBase::Init(config_);
        planner_dispatcher_->Init(){//file in/apollo/modules/planning/planner/planner_dispatcher.h
            RegisterPlanners{//file in/apollo/modules/planning/planner/planner_dispatcher.cc
                 planner_factory_.Register(PlannerType::PUBLIC_ROAD, []() -> Planner* {
                    return new PublicRoadPlanner();});
            }
        }//planner_dispatcher_->Init
        hdmap_ = HDMapUtil::BaseMapPtr();
        planner_ = planner_dispatcher_->DispatchPlanner(){//file in/apollo/modules/planning/planner/on_lane_planner_dispatcher.cc
            PlanningConfig planning_config;
            bool res_load_config = apollo::cyber::common::GetProtoFromFile(
                FLAGS_planning_config_file, &planning_config);//Proto文件里有写Planning_type,常用的是PUBLIC_ROAD
            return planner_factory_.CreateObject(
                    planning_config.standard_planning_config().planner_type(0));//这里return的是Public_Road_Planner类
        }//planner_dispatcher_->DispatchPlanner
        return planner_->Init(config_){//file in/apollo/modules/planning/planner/public_road/public_road_planner.cc
            scenario_manager_.Init(supported_scenarios){//secnario_manager_是planner的成员参数，/apollo/modules/planning/scenarios/scenario_manager.cc
                RegisterScenarios(){
                    // lane_follow
                    CHECK(Scenario::LoadConfig(FLAGS_scenario_lane_follow_config_file,
                             &config_map_[ScenarioConfig::LANE_FOLLOW]));
                }
                default_scenario_type_ = ScenarioConfig::LANE_FOLLOW;//默认场景是LANE_FOLLOW
                current_scenario_ = CreateScenario(default_scenario_type_){
                    switch (scenario_type) {
                        case ScenarioConfig::LANE_FOLLOW:
                            ptr.reset(new lane_follow::LaneFollowScenario(config_map_[scenario_type],
                                                    &scenario_context_));
                            break;
                    }//switch
                }//CreateScenario
            }//scenario_manager_.Init
        }//planner_->Init
    }//planning_base_->Init
    planning_writer_ =
        node_->CreateWriter<ADCTrajectory>(FLAGS_planning_trajectory_topic);
}//PlanningComponent::Init
```
上述所列伪代码就是整个Init函数调用的层次关系。接下来介绍算法调度过程。
## 3. planning算法的调用步骤
算法调度使用函数PlanningComponent::Proc(const std::shared_ptr<prediction::PredictionObstacles>&
prediction_obstacles,
const std::shared_ptr<canbus::Chassis>& chassis,
const std::shared_ptr<localization::LocalizationEstimate>&
localization_estimate)
整体结构如下：
```C++
bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
    CheckRerouting();
    if(FLAGS_use_case_plan){//一般是false
    }else{
        planning_base_->RunOnce(local_view_, &adc_trajectory_pb){//file in/apollo/modules/planning/on_lane_planning.cc
            Status status = VehicleStateProvider::Instance()->Update(
                *local_view_.localization_estimate, *local_view_.chassis);//更新车辆状态
            reference_line_provider_->UpdateVehicleState(vehicle_state);//reference_line是一条长度大概200多米的介于全局routing和局部路径规划之间的参考线。
            status = Plan(start_timestamp, stitching_trajectory, ptr_trajectory_pb){//file in/apollo/modules/planning/planner/public_road/public_road_planner.cc
                scenario_manager_.Update(planning_start_point, *frame){//file in/apollo/modules/planning/scenarios/scenario_manager.cc
                    Observe(frame);
                    ScenarioDispatch(ego_point, frame){//这里面主要是场景切换的部分，通过各种判断切换scenario_type,最后如果场景有变化，建立新的场景
                        UpdatePlanningContext(frame, scenario_type);
                        if (current_scenario_->scenario_type() != scenario_type) {
                            current_scenario_ = CreateScenario(scenario_type);
                        }//if
                    }//ScenarioDispatch
                }//scenario_manager_.Update
                scenario_ = scenario_manager_.mutable_scenario();//这里从scenario_manager_里读取当前场景
                auto result = scenario_->Process(planning_start_point, frame){//当前场景的计算，file in/apollo/modules/planning/scenarios/scenario.cc
                    auto ret = current_stage_->Process(planning_init_point, frame){//不同的stage有不同的文件，先看了lane_follow_stage.file in/apollo/modules/planning/scenarios/lane_follow/lane_follow_stage.cc
                        auto cur_status =PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);//这里面就比较细了，在下一章讲
                    }//current_stage_->Process
                }//scenario_->Process
            }//Plan
            if (result == scenario::Scenario::STATUS_DONE) {//如果运算好了，更新场景
                // only updates scenario manager when previous scenario's status is
                // STATUS_DONE
                scenario_manager_.Update(planning_start_point, *frame);
            }//if
            //后面还运行了例如planning_smoother_.Smooth()等函数
            //
        }//planning_base_->RunOnce
    }//else
 }//Proc
```
##  4. scenarios内部详解
