import Foundation
@main struct PhoneFrameOrderTests {
 static func main() {
  var order=PhoneFrameOrder()
  precondition(order.accept(100,notBefore:90))
  precondition(!order.accept(100,notBefore:90))
  precondition(!order.accept(99,notBefore:90))
  precondition(order.lastTimestamp==100)
  precondition(order.accept(101,notBefore:90))
  precondition(!order.accept(.nan,notBefore:90))
  precondition(!order.accept(.infinity,notBefore:90))
  precondition(!order.accept(199,notBefore:200))
  precondition(order.accept(201,notBefore:200))
  precondition(order.accept(50,notBefore:40))
  print("AR frame ordering: duplicates, late callbacks, invalid time and session reset passed")
 }
}
