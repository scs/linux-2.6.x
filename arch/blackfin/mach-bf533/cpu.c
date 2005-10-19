
/*
 * cpu-bf533.c: clock scaling for the bf533
 *
 * Developers Manual" (available for free from Intel).
 *
 */
 /*
 * File:         arch/blackfin/mach-bf533/cpu.c
 * Based on:
 * Author:	michael.kang@analog.com
 *
 * Created:
 * Description: clock scaling for the bf533
 *
 * Rev:
 *
 * Modified:
 *
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.
 * If not, write to the Free Software Foundation,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <asm/dpmc.h>
#include <linux/fs.h>

#undef BF533_CPU_DEBUG

//CONFIG_CLKIN_HZ=11059200
#define VCO5 (CONFIG_CLKIN_HZ*45)/*497664000*/
#define VCO4 (CONFIG_CLKIN_HZ*36)/*398131200*/
#define VCO3 (CONFIG_CLKIN_HZ*27)/*298598400*/
#define VCO2 (CONFIG_CLKIN_HZ*18) /*199065600*/
#define VCO1 (CONFIG_CLKIN_HZ*9) /*99532800*/
#define VCO(x) VCO##x

#define FREQ(x) {VCO(x),VCO(x)/4},{VCO(x),VCO(x)/2},{VCO(x),VCO(x)}
/* frequency */
static struct cpufreq_frequency_table bf533_freq_table[] = {
	FREQ(1),
	FREQ(3),
	{VCO4,VCO4/2},{VCO4,VCO4},
	FREQ(5),
        {0,CPUFREQ_TABLE_END},
};

extern struct file_operations dpmc_fops;

/*
 * dpmc_fops->ioctl() 
 * static int dpmc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
 */

extern unsigned long get_cclk();
static int bf533_getfreq(unsigned int cpu)
{
	unsigned long cclk_mhz,vco_mhz;
	
	/*The driver only support single cpu*/
	if(cpu == 0)
		dpmc_fops.ioctl(NULL,NULL,IOCTL_GET_CORECLOCK,&cclk_mhz);
	else
		cclk_mhz = -1;
        return cclk_mhz;
}
/**/
static int bf533_target(struct cpufreq_policy *policy,
			 unsigned int target_freq,
			 unsigned int relation)
{
	unsigned long cclk_mhz ;
	unsigned long vco_mhz;
	unsigned long flags;
	unsigned int index,vco_index;
	int i;	
	
	struct cpufreq_freqs freqs;
	if(cpufreq_frequency_table_target(policy, bf533_freq_table,target_freq, relation, & index))
		return -EINVAL;
	cclk_mhz = bf533_freq_table[index].frequency;
	vco_mhz = bf533_freq_table[index].index;
	
	dpmc_fops.ioctl(NULL,NULL,IOCTL_CHANGE_FREQUENCY,&vco_mhz);
	freqs.old = bf533_getfreq(0);
        freqs.new = cclk_mhz;
        freqs.cpu = 0;

#if defined(BF533_CPU_DEBUG)
	printk("cclk begin change to cclk %d,vco=%d,index=%d,target=%d,oldfreq=%d\n",cclk_mhz,vco_mhz,index,target_freq,freqs.old);
#endif
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	local_irq_save(flags);
	dpmc_fops.ioctl(NULL,NULL,IOCTL_SET_CCLK,&cclk_mhz);
	local_irq_restore(flags);
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	
	 vco_mhz = get_vco();
        cclk_mhz = get_cclk();
	return 0;
}
/* make sure that only the "userspace" governor is run -- anything else wouldn't make sense on
 * this platform, anyway.
 */
static int bf533_verify_speed(struct cpufreq_policy *policy)
{
        return cpufreq_frequency_table_verify( policy, &bf533_freq_table);
}


static int __init __bf533_cpu_init(struct cpufreq_policy *policy)
{
	int result;
	
	if (policy->cpu != 0)
		return -EINVAL;
	
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;
	
	policy->cpuinfo.transition_latency = CPUFREQ_ETERNAL;
	/*Now ,only support one cpu*/
	policy->cur = bf533_getfreq(0);
	cpufreq_frequency_table_get_attr(bf533_freq_table, policy->cpu);
	return cpufreq_frequency_table_cpuinfo(policy, bf533_freq_table);
}

static struct freq_attr* bf533_freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver bf533_driver = {
	.verify		= bf533_verify_speed,
	.target		= bf533_target,
	.get		= bf533_getfreq,
	.init		= __bf533_cpu_init,
	.name		= "bf533",
	.owner   =  THIS_MODULE,
	.attr		= bf533_freq_attr,
};

static int __init bf533_cpu_init(void)
{
	//MISC_MAJOR, DPMC_MINOR
        return(cpufreq_register_driver(&bf533_driver));
}

static void __exit bf533_cpu_exit(void)
{
        cpufreq_unregister_driver(&bf533_driver);
}

MODULE_AUTHOR ("Mickael Kang");
MODULE_DESCRIPTION ("cpufreq driver for BF533 CPU");
MODULE_LICENSE ("GPL");

module_init(bf533_cpu_init);
module_exit(bf533_cpu_exit);
