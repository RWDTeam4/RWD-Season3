#ifndef SPEED_H
#define SPEED_H

class Speed {
  public:
    class ConstPtr {
      public:
        ConstPtr(Speed& x): parent(x) {}
      private:
        Speed& parent;
    };

  
    int speed;
    const ConstPtr pointer = (*this);
};

#endif
