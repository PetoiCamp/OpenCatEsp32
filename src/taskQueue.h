long taskTimer = 0;
long taskInterval = -1;
class Task {
public:
  char tkn;
  char *parameters;
  int paraLength;
  int dly;
  template<typename T>
  Task(char t, T *p, int d = 0)
    : tkn{ t }, dly{ d } {
    paraLength = (tkn >= 'A' && tkn <= 'Z') ? strlenUntil(p, '~') : strlen((char *)p);
    parameters = new char[paraLength + 1];
    arrayNCPY(parameters, p, paraLength);
    parameters[paraLength] = (tkn >= 'A' && tkn <= 'Z') ? '~' : '\0';
    // PTL("create task ");
    // info();
  };
  ~Task() {
    // if (paraLength)
    delete[] parameters;
  };
  void info() {
    printCmdByType(tkn, parameters);
  }
};

class TaskQueue : public QList<Task *> {
public:
  Task *lastTask;
  TaskQueue() {
    PTLF("TaskQ");
    lastTask = NULL;
  };
  template<typename T>
  void addTask(char t, T *p, int d = 0) {
    // PTH("add ", p);
    this->push_back(new Task(t, p, d));
  }
  template<typename T>
  void addTaskToFront(char t, T *p, int d = 0) {
    PTH("add front", p);
    this->push_front(new Task(t, p, d));
  }
  void createTask() {  // this is an example task
    this->addTask('k', "vtF", 2000);
    this->addTask('k', "up");
  }
  bool cleared() {
    return this->size() == 0 && long(millis() - taskTimer) > taskInterval;
  }
  void loadTaskInfo(Task *t) {
    token = t->tkn;
    cmdLen = t->paraLength;
    taskInterval = t->dly;
    strcpy(lastCmd, newCmd);
    arrayNCPY(newCmd, t->parameters, cmdLen);
    newCmd[cmdLen] = (token >= 'A' && token <= 'Z') ? '~' : '\0';
    taskTimer = millis();
    newCmdIdx = 5;
  }
  void popTask() {
    if (long(millis() - taskTimer) > taskInterval) {
      if (this->size() > 0) {
        loadTaskInfo(this->front());
        this->pop_front();
        // PTL("Use pop ");
      }
    }
  }
};
TaskQueue *tQueue;
