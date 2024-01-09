def test_convert_c(fahrenheit):
c = float(fahrenheit)
c = (c-32)*5/9
return(c)
def test_convert_f(celsius):
f = float(celsius)
f = (f * 9/5) + 32.
return(f)
