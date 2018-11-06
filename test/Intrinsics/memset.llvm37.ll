; REQUIRES: geq-llvm-3.7
; RUN: %llvmas %s -o=%t.bc
; RUN: rm -rf %t.klee-out
; RUN: %klee -exit-on-error --optimize=true --suppress-external-warnings=false --output-dir=%t.klee-out %t.bc > %t2 2>&1
; RUN: grep PASS %t2
; RUN: not grep "undefined reference to function: memset" %t2
; RUN: not grep "calling external: memset" %t2
; RUN: FileCheck %s --input-file=%t.klee-out/assembly.ll
target datalayout = "e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

declare i32 @puts(i8*)

@.passstr = private constant [5 x i8] c"PASS\00", align 1
@.failstr = private constant [5 x i8] c"FAIL\00", align 1

define i32 @main() optnone noinline nounwind uwtable {
entry:
  %s = alloca i32, align 4
  %p = bitcast i32* %s to i8*
  ; CHECK: call i8* @memset(
  call void @llvm.memset.p0i8.i64(i8* %p, i8 1, i64 4, i32 4, i1 false)
  %v = load i32, i32* %s, align 4
  %r = icmp eq i32 %v, 16843009
  br i1 %r, label %bbtrue, label %bbfalse

bbtrue:
  %res = call i32 @puts(i8* getelementptr inbounds ([5 x i8], [5 x i8]* @.passstr, i64 0, i64 0)) nounwind
  ret i32 0

bbfalse:
  %res2 = call i32 @puts(i8* getelementptr inbounds ([5 x i8], [5 x i8]* @.failstr, i64 0, i64 0)) nounwind
  ret i32 0
}

declare void @llvm.memset.p0i8.i64(i8*, i8, i64, i32, i1) nounwind
