$Ts = 1.0 / 30.0

# A = [1, Ts, 0.5*Ts^2; 0, 1, Ts; 0, 0, 1]
$a11 = 1.0; $a12 = $Ts; $a13 = 0.5 * $Ts * $Ts
$a21 = 0.0; $a22 = 1.0; $a23 = $Ts
$a31 = 0.0; $a32 = 0.0; $a33 = 1.0

# ATA:
$m11 = $a11*$a11 + $a21*$a21 + $a31*$a31
$m12 = $a11*$a12 + $a21*$a22 + $a31*$a32
$m13 = $a11*$a13 + $a21*$a23 + $a31*$a33

$m21 = $m12
$m22 = $a12*$a12 + $a22*$a22 + $a32*$a32
$m23 = $a12*$a13 + $a22*$a23 + $a32*$a33

$m31 = $m13
$m32 = $m23
$m33 = $a13*$a13 + $a23*$a23 + $a33*$a33

# tr, c2, det
$tr = $m11 + $m22 + $m33
$c2 = ($m11*$m22 - $m12*$m21) + ($m11*$m33 - $m13*$m31) + ($m22*$m33 - $m23*$m32)
$det = $m11*($m22*$m33 - $m23*$m32) - $m12*($m21*$m33 - $m23*$m31) + $m13*($m21*$m32 - $m22*$m31)

$p = $tr; $q = $c2; $r = $det
$a = (3.0*$q - $p*$p)/3.0
$b = (2.0*$p*$p*$p - 9.0*$p*$q + 27.0*$r)/27.0

$theta = [Math]::Acos(-$b / (2.0 * [Math]::Sqrt(- ($a/3.0)*($a/3.0)*($a/3.0))))
$r_dep = 2.0 * [Math]::Sqrt(-$a / 3.0)
$t1 = $r_dep * [Math]::Cos($theta / 3.0)
$t2 = $r_dep * [Math]::Cos(($theta + 2.0*[Math]::PI) / 3.0)
$t3 = $r_dep * [Math]::Cos(($theta + 4.0*[Math]::PI) / 3.0)
$l1 = $t1 + $p / 3.0
$l2 = $t2 + $p / 3.0
$l3 = $t3 + $p / 3.0

$s1 = [Math]::Sqrt($l1)
$s2 = [Math]::Sqrt($l2)
$s3 = [Math]::Sqrt($l3)

$s_max = [Math]::Max($s1, [Math]::Max($s2, $s3))
$s_min = [Math]::Min($s1, [Math]::Min($s2, $s3))
$cond = $s_max / $s_min

Write-Output "Singular values of A_kf: $s1, $s2, $s3"
Write-Output "Condition number kappa2(A_kf): $cond"
