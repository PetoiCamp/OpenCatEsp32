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
  void createTask() {  // use 'q' to start the sequence.
                       // add subToken followed by the subCommand
                       // use ':' to add the delay time (mandatory)
                       // add '~' to end the sub command
                       // example: qk sit:1000~m 8 0 8 -30 8 0:500~
    // PTL(newCmd);
    char *sub;
    sub = strtok(newCmd, ":");
    while (sub != NULL) {
      char subToken = *sub++;
      while (*sub == ' ' || *sub == '\t')  //remove the space between the subToken and the subCommand
        sub++;
      if (*sub == '\0')
        break;
      int subLen = strlen(sub);
      PTHL("sublen", subLen);
      char *subCmd = new char[subLen + 1];
      strcpy(subCmd, sub);
      sub = strtok(NULL, ">");
      int subDuration = atoi(sub);
      sub = strtok(NULL, ":");
      PTH(subToken, subCmd);
      PTHL(": ", subDuration);
      this->addTask(subToken, subCmd, subDuration);
      delete[] subCmd;
    }
    // this->addTask('k', "up");
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
