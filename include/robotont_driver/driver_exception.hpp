#ifndef ROBOTONT_DRIVER_EXCEPTION_HPP
#define ROBOTONT_DRIVER_EXCEPTION_HPP

struct DriverException : public std::exception
{
  std::string desc_;
  DriverException(std::string desc) : desc_(desc)
  {
  }

  ~DriverException() throw()
  {
  }

  const char* what() const throw()
  {
    return desc_.c_str();
  }
};

#endif
