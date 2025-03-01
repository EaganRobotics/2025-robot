// Package == folder
package frc.robot.Robot25;

// Imports

// Class declaration
public class Test {

  // Constants
  public static final String SOME_CONSTANT_NAME = "this is a constant and i cant change it";

  // Static variables
  public static int numInstances = 0;

  // Fields
  private int name = 2;
  public String test = "blah";
  String last = "dd";

  // Constructors
  public Test(int someNumber, String someString) {
    name = someNumber;
    test = someString;
  }

  private Test() {
    test = "secret";
    name = 0;
  }

  // Instance functions: methods
  public void funcName() {
    var i = 2;
    System.out.println("hello" + i);
  }

  // Static functions
  public static void staticFunction() {
    System.out.println("This is a static function");
  }
}

// Test objectName = new Test(1, "string");
// Test object2 = new Test(1, "string");
// Test.numInstances = 3;
// Test.staticFunction();
// objectName.test = "Griffin"
// objectName.funcName()
// X objectName.numInstances = 4; does not work
// X objectName.staticFunction(); does not work
// object2.test = "Reece"
// object2.funcName()
